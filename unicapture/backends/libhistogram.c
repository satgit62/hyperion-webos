#define _DEFAULT_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "log.h"
#include "unicapture.h"

#define DEV_MODE_PATH "/var/luna/preferences/vt_devmode"
#define SPATIAL_WIDTH 160
#define SPATIAL_HEIGHT 90
#define SPATIAL_PIXELS (SPATIAL_WIDTH * SPATIAL_HEIGHT)
#define MAX_PIXEL_REFS 4

typedef struct {
    uint32_t id;
    uint16_t bins[3][256];
} Histogram;

typedef struct {
    double position;
    unsigned char region;
} RimPoint;

typedef struct {
    unsigned char region;
    double x0;
    double y0;
    double x1;
    double y1;
} RegionRect;

typedef struct {
    unsigned char count;
    unsigned char regions[MAX_PIXEL_REFS];
    uint16_t weights[MAX_PIXEL_REFS];
} PixelMix;

static const RimPoint g4_rim[] = {
    { 0.2078431373, 7 },  { 0.3176470588, 3 },  { 0.5490196078, 11 },
    { 0.7529411765, 12 }, { 0.8392156863, 13 }, { 1.4941176471, 10 },
    { 2.1843137255, 9 },  { 2.5490196078, 1 },  { 2.8666666667, 5 },
    { 3.4000000000, 15 }
};

static const RimPoint c_series_rim[] = {
    { 0.1250000000, 7 },  { 0.3750000000, 3 },  { 0.6250000000, 11 },
    { 0.8750000000, 12 }, { 1.2500000000, 13 }, { 1.7500000000, 10 },
    { 2.1250000000, 9 },  { 2.3750000000, 1 },  { 2.6250000000, 5 },
    { 2.8750000000, 4 },  { 3.2500000000, 15 }, { 3.7500000000, 14 }
};

/* 16 gemessene Panel-Rechtecke inkl. kompletter Unterkante fuer 4-seitiges Ambilight */
static const RegionRect default_rects[] = {
    { 7,  0.09, 0.02, 0.32, 0.35 },
    { 3,  0.00, 0.00, 0.45, 0.20 },
    { 0,  0.31, 0.02, 0.69, 0.15 },
    { 11, 0.35, 0.00, 0.65, 0.16 },
    { 12, 0.68, 0.00, 1.00, 0.35 },
    { 13, 0.71, 0.09, 1.00, 0.36 },
    { 10, 0.80, 0.22, 1.00, 0.72 },
    { 9,  0.70, 0.62, 1.00, 1.00 },
    { 1,  0.15, 0.76, 0.85, 1.00 },
    { 5,  0.00, 0.63, 0.29, 1.00 },
    { 15, 0.01, 0.24, 0.11, 0.66 },
    { 6,  0.00, 0.21, 0.20, 0.72 },
    { 4,  0.22, 0.02, 0.48, 0.96 },
    { 2,  0.31, 0.28, 0.69, 0.66 },
    { 8,  0.52, 0.02, 0.78, 0.96 },
    { 14, 0.00, 0.00, 1.00, 1.00 }
};

typedef struct {
    void* vt_lib;
    void* (*vt_init)(bool (*)(void));
    int (*vt_preset)(void*, int);
    int (*vt_get)(void*, Histogram*);
    int (*vt_finish)(void*);

    void* handle;
    pid_t worker_pid;
    int worker_fd;
    uint32_t fps;
    int preset;
    unsigned settling;
    unsigned letterbox_mode;
    int pending_letterbox;
    uint64_t candidate_since;
    uint64_t next_border_probe;

    bool average_mode;
    bool rectangle_mode;
    float srgb_to_lin[256];
    uint8_t lin_to_srgb[4096];

    uint8_t* rgb_buffer;
    PixelMix mixes[SPATIAL_PIXELS];
} histogram_backend_state_t;

static volatile sig_atomic_t g_policy_revoked = 0;

static bool policy_callback(void)
{
    g_policy_revoked = 1;
    return true;
}

static uint64_t now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000ULL + (uint64_t)t.tv_nsec / 1000000ULL;
}

static void init_color_luts(histogram_backend_state_t* self)
{
    for (int i = 0; i < 256; i++) {
        double s = i / 255.0;
        double lin = (s <= 0.04045) ? (s / 12.92) : pow((s + 0.055) / 1.055, 2.4);
        self->srgb_to_lin[i] = (float)lin;
    }
    for (int i = 0; i < 4096; i++) {
        double lin = i / 4095.0;
        double s = (lin <= 0.0031308) ? (12.92 * lin) : (1.055 * pow(lin, 1.0 / 2.4) - 0.055);
        int v = (int)(s * 255.0 + 0.5);
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        self->lin_to_srgb[i] = (uint8_t)v;
    }
}

static uint8_t encode_linear(const histogram_backend_state_t* self, float lin)
{
    if (lin <= 0.0f) return 0;
    if (lin >= 1.0f) return 255;
    int idx = (int)(lin * 4095.0f + 0.5f);
    return self->lin_to_srgb[idx];
}

static double screen_perimeter(double x, double y)
{
    double distances[] = { y, 1.0 - x, 1.0 - y, x };
    double positions[] = { x, 1.0 + y, 3.0 - x, 4.0 - y };
    unsigned edge = 0;
    for (unsigned i = 1; i < 4; i++) {
        if (distances[i] < distances[edge]) edge = i;
    }
    return positions[edge] >= 4.0 ? positions[edge] - 4.0 : positions[edge];
}

static void build_spatial_mixes(histogram_backend_state_t* self, const RimPoint* rim, unsigned rim_count)
{
    for (unsigned y = 0; y < SPATIAL_HEIGHT; y++) {
        double py = (y + 0.5) / SPATIAL_HEIGHT;
        for (unsigned x = 0; x < SPATIAL_WIDTH; x++) {
            double px = (x + 0.5) / SPATIAL_WIDTH;
            PixelMix* m = &self->mixes[y * SPATIAL_WIDTH + x];
            memset(m, 0, sizeof(*m));

            if (self->average_mode) {
                m->count = 1;
                m->regions[0] = 0;
                m->weights[0] = 65535;
                continue;
            }

            if (self->rectangle_mode) {
                unsigned cand_r[16];
                double cand_w[16];
                unsigned cand_n = 0;

                for (unsigned i = 0; i < 16; i++) {
                    const RegionRect* rc = &default_rects[i];
                    if (px >= rc->x0 && px <= rc->x1 && py >= rc->y0 && py <= rc->y1) {
                        double area = (rc->x1 - rc->x0) * (rc->y1 - rc->y0);
                        if (area < 1e-6) area = 1e-6;
                        cand_r[cand_n] = rc->region;
                        cand_w[cand_n] = 1.0 / area;
                        cand_n++;
                    }
                }

                if (cand_n == 0) {
                    /* Nearest-Center Fallback */
                    unsigned best_r = 0;
                    double best_d2 = 1e9;
                    for (unsigned i = 0; i < 16; i++) {
                        double cx = 0.5 * (default_rects[i].x0 + default_rects[i].x1);
                        double cy = 0.5 * (default_rects[i].y0 + default_rects[i].y1);
                        double d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy);
                        if (d2 < best_d2) {
                            best_d2 = d2;
                            best_r = default_rects[i].region;
                        }
                    }
                    m->count = 1;
                    m->regions[0] = (unsigned char)best_r;
                    m->weights[0] = 65535;
                    continue;
                }

                /* Top-4 schwerste (kleinste) Rechtecke waehlen und normalisieren */
                for (unsigned a = 0; a < cand_n; a++) {
                    for (unsigned b = a + 1; b < cand_n; b++) {
                        if (cand_w[b] > cand_w[a]) {
                            double tw = cand_w[a]; cand_w[a] = cand_w[b]; cand_w[b] = tw;
                            unsigned tr = cand_r[a]; cand_r[a] = cand_r[b]; cand_r[b] = tr;
                        }
                    }
                }

                unsigned take = cand_n > MAX_PIXEL_REFS ? MAX_PIXEL_REFS : cand_n;
                double wsum = 0.0;
                for (unsigned i = 0; i < take; i++) wsum += cand_w[i];

                m->count = (unsigned char)take;
                unsigned acc = 0;
                for (unsigned i = 0; i < take; i++) {
                    m->regions[i] = (unsigned char)cand_r[i];
                    if (i + 1 == take) {
                        m->weights[i] = (uint16_t)(65535U - acc);
                    } else {
                        unsigned w = (unsigned)(cand_w[i] / wsum * 65535.0 + 0.5);
                        m->weights[i] = (uint16_t)w;
                        acc += w;
                    }
                }
            } else {
                /* Klassische Perimeter-Interpolation (Rim) */
                double t = screen_perimeter(px, py);
                unsigned right = 0;
                while (right < rim_count && rim[right].position <= t) right++;
                right %= rim_count;
                unsigned left = (right + rim_count - 1) % rim_count;
                double span = rim[right].position - rim[left].position;
                double dist = t - rim[left].position;
                if (span <= 0.0) span += 4.0;
                if (dist < 0.0) dist += 4.0;
                unsigned wb = (unsigned)(dist / span * 65535.0 + 0.5);
                m->count = 2;
                m->regions[0] = rim[left].region;
                m->regions[1] = rim[right].region;
                m->weights[0] = (uint16_t)(65535U - wb);
                m->weights[1] = (uint16_t)wb;
            }
        }
    }
}

static bool enter_developer_mode(int* guard_pipe, pid_t* guard_pid)
{
    *guard_pipe = -1;
    *guard_pid = -1;

    int marker = open(DEV_MODE_PATH, O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0600);
    if (marker < 0) return errno == EEXIST;

    struct stat owned;
    if (fstat(marker, &owned) != 0) {
        close(marker);
        unlink(DEV_MODE_PATH);
        return false;
    }
    close(marker);

    int p[2];
    if (pipe(p) != 0) {
        unlink(DEV_MODE_PATH);
        return false;
    }

    pid_t child = fork();
    if (child < 0) {
        close(p[0]);
        close(p[1]);
        unlink(DEV_MODE_PATH);
        return false;
    }

    if (child == 0) {
        close(p[1]);
        struct pollfd fd = { p[0], POLLIN | POLLHUP, 0 };
        poll(&fd, 1, 10000);
        close(p[0]);

        struct stat current;
        int rc = 0;
        if (lstat(DEV_MODE_PATH, &current) == 0 &&
            current.st_dev == owned.st_dev &&
            current.st_ino == owned.st_ino) {
            rc = unlink(DEV_MODE_PATH);
        }
        _exit(rc == 0 ? 0 : 1);
    }

    close(p[0]);
    *guard_pipe = p[1];
    *guard_pid = child;
    return true;
}

static void leave_developer_mode(int* guard_pipe, pid_t* guard_pid)
{
    if (*guard_pipe < 0) return;
    close(*guard_pipe);
    *guard_pipe = -1;

    int status;
    while (waitpid(*guard_pid, &status, 0) < 0) {
        if (errno != EINTR) break;
    }
    *guard_pid = -1;
}

static bool histogram_populated(const Histogram hist[16])
{
    for (int i = 0; i < 16; i++) {
        for (int c = 0; c < 3; c++) {
            for (int b = 0; b < 256; b++) {
                if (hist[i].bins[c][b] != 0) return true;
            }
        }
    }
    return false;
}

static void dark_color_filter(unsigned char color[3])
{
    unsigned peak = color[0];
    for (unsigned c = 1; c < 3; c++) {
        if (color[c] > peak) peak = color[c];
    }
    if (peak >= 18) return;
    unsigned gain = peak > 6 ? peak - 6 : 0;
    for (unsigned c = 0; c < 3; c++) {
        color[c] = (unsigned char)((color[c] * gain + 8) / 16);
    }
}

static int letterbox_candidate(const unsigned black[16], unsigned char color[16][3])
{
    const unsigned interior[] = { 2, 6, 10, 15 };
    unsigned lit = 0, interior_black = 0, interior_peak = 0;
    for (unsigned i = 0; i < 4; i++) {
        unsigned r = interior[i];
        unsigned peak = color[r][0];
        for (unsigned c = 1; c < 3; c++) {
            if (color[r][c] > peak) peak = color[r][c];
        }
        if (black[r] < 20 && peak > 35) {
            lit++;
            interior_black += black[r];
            interior_peak += peak;
        }
    }
    if (lit < 2) return -1;
    unsigned top = (black[3] + black[11] + black[12]) / 3;
    unsigned bottom = (black[1] + black[5] + black[9]) / 3;
    if (top >= 55 && bottom >= 25) return 1;

    const unsigned top_regions[] = { 3, 11, 12 };
    const unsigned bottom_regions[] = { 1, 5, 9 };
    unsigned top_support = 0, bottom_support = 0, top_peak = 0;
    for (unsigned i = 0; i < 3; i++) {
        unsigned r = top_regions[i];
        unsigned peak = color[r][0];
        for (unsigned c = 1; c < 3; c++) {
            if (color[r][c] > peak) peak = color[r][c];
        }
        top_peak += peak;
        top_support += black[r] >= 12 ? 1U : 0U;
        bottom_support += black[bottom_regions[i]] >= 8 ? 1U : 0U;
    }
    unsigned baseline = interior_black / lit;
    if (top_support == 3 && bottom_support >= 2 && top >= 12 && bottom >= 10 &&
        top >= baseline + 8 && bottom >= baseline + 8 &&
        (unsigned long)top_peak * lit * 100 <= (unsigned long)interior_peak * 3 * 60) {
        return 1;
    }
    if (top < 20 && bottom < 20) return 0;
    return -1;
}

int capture_init(cap_backend_config_t* config, void** state_p)
{
    histogram_backend_state_t* self = calloc(1, sizeof(histogram_backend_state_t));
    if (!self) return -1;

    self->vt_lib = dlopen("libvtcapture.so.1", RTLD_NOW);
    if (!self->vt_lib) {
        ERR("histogram_backend: dlopen libvtcapture.so.1 failed: %s", dlerror());
        free(self);
        return -2;
    }

    *(void**)(&self->vt_init) = dlsym(self->vt_lib, "vtCapture_initHistogram");
    *(void**)(&self->vt_preset) = dlsym(self->vt_lib, "vtCapture_setPresetHistogram");
    *(void**)(&self->vt_get) = dlsym(self->vt_lib, "vtCapture_getHistogram");
    *(void**)(&self->vt_finish) = dlsym(self->vt_lib, "vtCapture_finalizeHistogram");

    if (!self->vt_init || !self->vt_preset || !self->vt_get || !self->vt_finish) {
        ERR("histogram_backend: missing vtCapture histogram symbols");
        dlclose(self->vt_lib);
        free(self);
        return -3;
    }

    self->rgb_buffer = calloc(SPATIAL_PIXELS * 3, 1);
    if (!self->rgb_buffer) {
        dlclose(self->vt_lib);
        free(self);
        return -4;
    }

    init_color_luts(self);

    const char* profile = getenv("HYPERION_HISTOGRAM_PROFILE");
    const char* geom = getenv("HYPERION_HISTOGRAM_GEOMETRY");

    self->average_mode = (profile != NULL && strcmp(profile, "average") == 0);
    self->rectangle_mode = (geom == NULL || strcmp(geom, "rim") != 0);

    const RimPoint* rim = g4_rim;
    unsigned rim_count = (unsigned)(sizeof(g4_rim) / sizeof(g4_rim[0]));
    if (profile != NULL && (strcmp(profile, "c2") == 0 || strcmp(profile, "c3") == 0)) {
        rim = c_series_rim;
        rim_count = (unsigned)(sizeof(c_series_rim) / sizeof(c_series_rim[0]));
    }

    build_spatial_mixes(self, rim, rim_count);

    self->fps = (config && config->fps > 0) ? config->fps : 30;
    self->worker_pid = -1;
    self->worker_fd = -1;
    self->pending_letterbox = -1;
    *state_p = self;
    INFO("histogram_backend: initialized (profile=%s, geometry=%s, linear-light)",
         profile ? profile : "default",
         self->average_mode ? "average" : (self->rectangle_mode ? "rectangles" : "rim"));
    return 0;
}



int capture_terminate(void* state);

int capture_cleanup(void* state)
{
    histogram_backend_state_t* self = (histogram_backend_state_t*)state;
    if (!self) return 0;
    capture_terminate(self);
    if (self->vt_lib) dlclose(self->vt_lib);
    free(self->rgb_buffer);
    free(self);
    return 0;
}

static void worker_loop(histogram_backend_state_t* self, int fd) {
    int gp = -1; pid_t gpid = -1;
    enter_developer_mode(&gp, &gpid);
    g_policy_revoked = 0;
    void* handle = self->vt_init(policy_callback);
    if (!handle) { ERR("histogram_backend: vtCapture_initHistogram failed in worker (errno=%d)", errno); leave_developer_mode(&gp, &gpid); close(fd); _exit(1); }
    usleep(150000);
    self->vt_preset(handle, 0);
    usleep(100000);
    leave_developer_mode(&gp, &gpid);
    uint32_t d = 1000000 / (self->fps > 0 ? self->fps : 30);
    Histogram hist[16];
    unsigned empty_ticks = 0;
    bool ever_populated = false;
    while (!g_policy_revoked) {
        usleep(d); memset(hist, 0, sizeof(hist));
        int rc = self->vt_get(handle, hist);
        if (rc == 0 && histogram_populated(hist)) {
            if (!ever_populated) INFO("histogram_backend: first active video histogram frame captured!");
            ever_populated = true;
            empty_ticks = 0;
            if (write(fd, hist, sizeof(hist)) != (ssize_t)sizeof(hist)) break;
        } else {
            empty_ticks++;
            if ((ever_populated && empty_ticks >= 120) || (!ever_populated && empty_ticks >= 600)) {
                INFO("histogram_backend: stream ended or inactive (rc=%d, ticks=%u), exiting DRM fallback", rc, empty_ticks);
                break;
            }
        }
    }
    self->vt_finish(handle); close(fd); _exit(0);
}
int capture_start(void* state) {
    histogram_backend_state_t* self = (histogram_backend_state_t*)state;
    if (self->worker_pid > 0) return 0;
    int fds[2]; if (pipe(fds) != 0) return -1;
    pid_t pid = fork();
    if (pid < 0) { close(fds[0]); close(fds[1]); return -1; }
    if (pid == 0) { close(fds[0]); worker_loop(self, fds[1]); _exit(0); }
    close(fds[1]);
    fcntl(fds[0], F_SETFL, fcntl(fds[0], F_GETFL, 0) | O_NONBLOCK);
    usleep(250000);
    int st = 0;
    if (waitpid(pid, &st, WNOHANG) == pid) { close(fds[0]); ERR("histogram_backend: worker exited immediately"); return -1; }
    self->worker_pid = pid; self->worker_fd = fds[0];
    INFO("histogram_backend: started (isolated worker pid=%d)", (int)pid);
    return 0;
}
int capture_terminate(void* state) {
    histogram_backend_state_t* self = (histogram_backend_state_t*)state;
    if (self->worker_fd >= 0) { close(self->worker_fd); self->worker_fd = -1; }
    if (self->worker_pid > 0) { kill(self->worker_pid, SIGTERM); int st = 0; waitpid(self->worker_pid, &st, 0); self->worker_pid = -1; }
    INFO("histogram_backend: terminated");
    return 0;
}
int capture_wait(void* state) { histogram_backend_state_t* self = (histogram_backend_state_t*)state; usleep(1000000 / self->fps); return 0; }
int capture_acquire_frame(void* state, frame_info_t* frame) {
    histogram_backend_state_t* self = (histogram_backend_state_t*)state;
    if (self->worker_pid <= 0 || self->worker_fd < 0) return -1;
    uint64_t now = now_ms(); (void)now;
    static Histogram last_hist[16];
    static bool has_last = false;
    static unsigned stale_reads = 0;
    Histogram hist[16];
    bool got_new = false;
    while (1) {
        Histogram tmp[16];
        ssize_t n = read(self->worker_fd, tmp, sizeof(tmp));
        if (n == (ssize_t)sizeof(tmp)) {
            memcpy(last_hist, tmp, sizeof(tmp));
            has_last = true;
            got_new = true;
            stale_reads = 0;
        } else if (n == 0) {
            has_last = false;
            stale_reads = 0;
            return -1;
        } else {
            break;
        }
    }
    if (!got_new) {
        if (!has_last) return -2;
        if (++stale_reads > 30) {
            has_last = false;
            stale_reads = 0;
            return -1;
        }
    }
    memcpy(hist, last_hist, sizeof(hist));
    unsigned black_percent[16];
    unsigned char srgb_eval[16][3];
    float lin_rgb[16][3];
    for (int i = 0; i < 16; i++) {
        black_percent[i] = 100;
        for (int c = 0; c < 3; c++) {
            uint64_t total = 0, black = 0;
            double lin_sum = 0.0;
            for (int b = 0; b < 256; b++) {
                uint16_t cnt = hist[i].bins[c][b];
                if (cnt == 0) continue;
                total += cnt;
                if (b <= 12) black += cnt;
                lin_sum += (double)cnt * self->srgb_to_lin[b];
            }
            unsigned fraction = total ? (unsigned)(100 * black / total) : 0;
            if (fraction < black_percent[i]) black_percent[i] = fraction;
            float mean_lin = total ? (float)(lin_sum / (double)total) : 0.0f;
            lin_rgb[i][c] = mean_lin;
            srgb_eval[i][c] = encode_linear(self, mean_lin);
        }
    }
    int candidate = letterbox_candidate(black_percent, srgb_eval);
    if (candidate < 0) self->pending_letterbox = -1;
    else if (candidate != self->pending_letterbox) { self->pending_letterbox = candidate; self->candidate_since = now; }
    else if ((unsigned)candidate != self->letterbox_mode && now - self->candidate_since >= 2000) self->letterbox_mode = (unsigned)candidate;

    if (self->letterbox_mode) {
        /* 21:9 Letterbox-Balken oben/unten ausblenden, ohne vt_preset(NULL) aufzurufen */
        srgb_eval[3][0] = srgb_eval[6][0];  srgb_eval[3][1] = srgb_eval[6][1];  srgb_eval[3][2] = srgb_eval[6][2];
        srgb_eval[11][0] = srgb_eval[2][0]; srgb_eval[11][1] = srgb_eval[2][1]; srgb_eval[11][2] = srgb_eval[2][2];
        srgb_eval[12][0] = srgb_eval[10][0];srgb_eval[12][1] = srgb_eval[10][1];srgb_eval[12][2] = srgb_eval[10][2];
        srgb_eval[5][0] = srgb_eval[6][0];  srgb_eval[5][1] = srgb_eval[6][1];  srgb_eval[5][2] = srgb_eval[6][2];
        srgb_eval[1][0] = srgb_eval[15][0]; srgb_eval[1][1] = srgb_eval[15][1]; srgb_eval[1][2] = srgb_eval[15][2];
        srgb_eval[9][0] = srgb_eval[10][0]; srgb_eval[9][1] = srgb_eval[10][1]; srgb_eval[9][2] = srgb_eval[10][2];
    }
    for (unsigned i = 0; i < 16; i++) {
        dark_color_filter(srgb_eval[i]);
        for (unsigned c = 0; c < 3; c++) lin_rgb[i][c] = self->srgb_to_lin[srgb_eval[i][c]];
    }
    if (self->average_mode) {
        float avg_r = 0.0f, avg_g = 0.0f, avg_b = 0.0f;
        for (unsigned i = 0; i < 16; i++) {
            avg_r += lin_rgb[i][0];
            avg_g += lin_rgb[i][1];
            avg_b += lin_rgb[i][2];
        }
        uint8_t r = encode_linear(self, avg_r / 16.0f);
        uint8_t g = encode_linear(self, avg_g / 16.0f);
        uint8_t b = encode_linear(self, avg_b / 16.0f);
        for (unsigned i = 0; i < SPATIAL_PIXELS; i++) {
            self->rgb_buffer[i * 3 + 0] = r;
            self->rgb_buffer[i * 3 + 1] = g;
            self->rgb_buffer[i * 3 + 2] = b;
        }
    } else {
        for (unsigned i = 0; i < SPATIAL_PIXELS; i++) {
            const PixelMix* m = &self->mixes[i];
            float acc_r = 0.0f, acc_g = 0.0f, acc_b = 0.0f;
            for (unsigned k = 0; k < m->count; k++) {
                float w = m->weights[k] / 65535.0f;
                unsigned reg = m->regions[k];
                acc_r += lin_rgb[reg][0] * w;
                acc_g += lin_rgb[reg][1] * w;
                acc_b += lin_rgb[reg][2] * w;
            }
            self->rgb_buffer[i * 3 + 0] = encode_linear(self, acc_r);
            self->rgb_buffer[i * 3 + 1] = encode_linear(self, acc_g);
            self->rgb_buffer[i * 3 + 2] = encode_linear(self, acc_b);
        }
    }
    frame->pixel_format = PIXFMT_RGB;
    frame->width = SPATIAL_WIDTH;
    frame->height = SPATIAL_HEIGHT;
    frame->planes[0].buffer = self->rgb_buffer;
    frame->planes[0].stride = SPATIAL_WIDTH * 3;
    return 0;
}

int capture_release_frame(void* state __attribute__((unused)), frame_info_t* frame __attribute__((unused)))
{
    return 0;
}
