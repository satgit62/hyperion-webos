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

typedef struct {
    uint32_t id;
    uint16_t bins[3][256];
} Histogram;

typedef struct {
    double position;
    unsigned char region;
} RimPoint;

typedef struct {
    unsigned char a;
    unsigned char b;
    unsigned weight;
} LedBlend;

static const RimPoint default_rim[] = {
    { 0.2078431373, 7 },  { 0.3176470588, 3 },  { 0.5490196078, 11 },
    { 0.7529411765, 12 }, { 0.8392156863, 13 }, { 1.4941176471, 10 },
    { 2.1843137255, 9 },  { 2.5490196078, 1 },  { 2.8666666667, 5 },
    { 3.4000000000, 15 }
};

typedef struct {
    void* vt_lib;
    void* (*vt_init)(bool (*)(void));
    int (*vt_preset)(void*, int);
    int (*vt_get)(void*, Histogram*);
    int (*vt_finish)(void*);

    void* handle;
    uint32_t fps;
    int preset;
    unsigned settling;
    unsigned letterbox_mode;
    int pending_letterbox;
    uint64_t candidate_since;
    uint64_t next_border_probe;

    uint8_t* rgb_buffer;
    LedBlend blends[SPATIAL_PIXELS];
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

static bool enter_developer_mode(int* guard_pipe, pid_t* guard_pid)
{
    *guard_pipe = -1;
    *guard_pid = -1;

    int marker = open(DEV_MODE_PATH, O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0600);
    if (marker < 0) {
        return errno == EEXIST;
    }

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
    if (*guard_pipe < 0) {
        return;
    }
    close(*guard_pipe);
    *guard_pipe = -1;

    int status;
    while (waitpid(*guard_pid, &status, 0) < 0) {
        if (errno != EINTR) {
            break;
        }
    }
    *guard_pid = -1;
}

static double screen_perimeter(double x, double y)
{
    double distances[] = { y, 1.0 - x, 1.0 - y, x };
    double positions[] = { x, 1.0 + y, 3.0 - x, 4.0 - y };
    unsigned edge = 0;
    for (unsigned i = 1; i < 4; i++) {
        if (distances[i] < distances[edge]) {
            edge = i;
        }
    }
    return positions[edge] >= 4.0 ? positions[edge] - 4.0 : positions[edge];
}

static LedBlend compute_layout_blend(double x, double y)
{
    double t = screen_perimeter(x, y);
    unsigned n = (unsigned)(sizeof(default_rim) / sizeof(default_rim[0]));
    unsigned right = 0;
    while (right < n && default_rim[right].position <= t) {
        right++;
    }
    right %= n;
    unsigned left = (right + n - 1) % n;
    double span = default_rim[right].position - default_rim[left].position;
    double distance = t - default_rim[left].position;
    if (span <= 0.0) {
        span += 4.0;
    }
    if (distance < 0.0) {
        distance += 4.0;
    }
    LedBlend b;
    b.a = default_rim[left].region;
    b.b = default_rim[right].region;
    b.weight = (unsigned)(distance / span * 65535.0 + 0.5);
    return b;
}

static unsigned char histogram_mean(const uint16_t bins[256])
{
    uint64_t count = 0;
    uint64_t weight = 0;
    for (unsigned b = 0; b < 256; b++) {
        count += bins[b];
        weight += (uint64_t)bins[b] * b;
    }
    return count ? (unsigned char)((weight + count / 2) / count) : 0;
}

static bool histogram_populated(const Histogram hist[16])
{
    for (int i = 0; i < 16; i++) {
        for (int c = 0; c < 3; c++) {
            for (int b = 0; b < 256; b++) {
                if (hist[i].bins[c][b] != 0) {
                    return true;
                }
            }
        }
    }
    return false;
}

static void dark_color_filter(unsigned char color[3])
{
    unsigned peak = color[0];
    for (unsigned c = 1; c < 3; c++) {
        if (color[c] > peak) {
            peak = color[c];
        }
    }
    if (peak >= 32) {
        return;
    }
    unsigned gain = peak > 16 ? peak - 16 : 0;
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
            if (color[r][c] > peak) {
                peak = color[r][c];
            }
        }
        if (black[r] < 20 && peak > 35) {
            lit++;
            interior_black += black[r];
            interior_peak += peak;
        }
    }
    if (lit < 2) {
        return -1;
    }
    unsigned top = (black[3] + black[11] + black[12]) / 3;
    unsigned bottom = (black[1] + black[5] + black[9]) / 3;
    if (top >= 55 && bottom >= 25) {
        return 1;
    }

    const unsigned top_regions[] = { 3, 11, 12 };
    const unsigned bottom_regions[] = { 1, 5, 9 };
    unsigned top_support = 0, bottom_support = 0, top_peak = 0;
    for (unsigned i = 0; i < 3; i++) {
        unsigned r = top_regions[i];
        unsigned peak = color[r][0];
        for (unsigned c = 1; c < 3; c++) {
            if (color[r][c] > peak) {
                peak = color[r][c];
            }
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
    if (top < 20 && bottom < 20) {
        return 0;
    }
    return -1;
}

int capture_init(cap_backend_config_t* config, void** state_p)
{
    histogram_backend_state_t* self = calloc(1, sizeof(histogram_backend_state_t));
    if (!self) {
        return -1;
    }

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

    for (unsigned y = 0; y < SPATIAL_HEIGHT; y++) {
        for (unsigned x = 0; x < SPATIAL_WIDTH; x++) {
            self->blends[y * SPATIAL_WIDTH + x] =
                compute_layout_blend((x + 0.5) / SPATIAL_WIDTH, (y + 0.5) / SPATIAL_HEIGHT);
        }
    }

    self->fps = (config && config->fps > 0) ? config->fps : 30;
    self->pending_letterbox = -1;
    *state_p = self;
    INFO("histogram_backend: initialized (%dx%d synthetic RGB)", SPATIAL_WIDTH, SPATIAL_HEIGHT);
    return 0;
}

int capture_cleanup(void* state)
{
    histogram_backend_state_t* self = (histogram_backend_state_t*)state;
    if (!self) {
        return 0;
    }
    if (self->handle) {
        self->vt_finish(self->handle);
        self->handle = NULL;
    }
    if (self->vt_lib) {
        dlclose(self->vt_lib);
    }
    free(self->rgb_buffer);
    free(self);
    return 0;
}

int capture_start(void* state)
{
    histogram_backend_state_t* self = (histogram_backend_state_t*)state;
    if (self->handle) {
        return 0;
    }

    int guard_pipe = -1;
    pid_t guard_pid = -1;
    if (!enter_developer_mode(&guard_pipe, &guard_pid)) {
        WARN("histogram_backend: could not create vt_devmode marker");
    }

    g_policy_revoked = 0;
    self->handle = self->vt_init(policy_callback);
    leave_developer_mode(&guard_pipe, &guard_pid);

    if (!self->handle) {
        ERR("histogram_backend: vtCapture_initHistogram failed");
        return -1;
    }

    self->preset = 0;
    self->letterbox_mode = 0;
    self->pending_letterbox = -1;
    self->settling = 1;
    self->vt_preset(self->handle, 0);

    INFO("histogram_backend: started");
    return 0;
}

int capture_terminate(void* state)
{
    histogram_backend_state_t* self = (histogram_backend_state_t*)state;
    if (self->handle) {
        self->vt_finish(self->handle);
        self->handle = NULL;
    }
    INFO("histogram_backend: terminated");
    return 0;
}

int capture_wait(void* state)
{
    histogram_backend_state_t* self = (histogram_backend_state_t*)state;
    if (g_policy_revoked) {
        WARN("histogram_backend: LG policy revoked capture, restarting...");
        return -99;
    }
    usleep(1000000 / self->fps);
    return 0;
}

int capture_acquire_frame(void* state, frame_info_t* frame)
{
    histogram_backend_state_t* self = (histogram_backend_state_t*)state;
    if (!self->handle || g_policy_revoked) {
        return -1;
    }

    uint64_t now = now_ms();

    if (self->settling > 0) {
        self->settling--;
        return -2;
    }

    if (self->letterbox_mode && self->preset == 1 && now >= self->next_border_probe) {
        if (self->vt_preset(self->handle, 0) == 0) {
            self->preset = 0;
            self->settling = 2;
        }
        return -2;
    }

    Histogram hist[16];
    memset(hist, 0, sizeof(hist));
    int rc = self->vt_get(self->handle, hist);
    if (rc != 0 || !histogram_populated(hist)) {
        return -3;
    }

    unsigned black_percent[16];
    unsigned char rgb[16][3];

    for (int i = 0; i < 16; i++) {
        black_percent[i] = 100;
        for (int c = 0; c < 3; c++) {
            uint64_t total = 0;
            uint64_t black = 0;
            for (int b = 0; b < 256; b++) {
                total += hist[i].bins[c][b];
                if (b <= 12) {
                    black += hist[i].bins[c][b];
                }
            }
            unsigned fraction = total ? (unsigned)(100 * black / total) : 0;
            if (fraction < black_percent[i]) {
                black_percent[i] = fraction;
            }
            rgb[i][c] = histogram_mean(hist[i].bins[c]);
        }
    }

    if (self->preset == 0) {
        int candidate = letterbox_candidate(black_percent, rgb);
        if (candidate < 0) {
            self->pending_letterbox = -1;
        } else if (candidate != self->pending_letterbox) {
            self->pending_letterbox = candidate;
            self->candidate_since = now;
        } else if ((unsigned)candidate != self->letterbox_mode &&
                   now - self->candidate_since >= 2000) {
            self->letterbox_mode = (unsigned)candidate;
        }

        if (self->letterbox_mode) {
            if (self->vt_preset(self->handle, 1) == 0) {
                self->preset = 1;
                self->settling = 2;
                self->next_border_probe = now + 2000;
            }
            return -2;
        }
    }

    for (unsigned i = 0; i < 16; i++) {
        dark_color_filter(rgb[i]);
    }

    for (unsigned i = 0; i < SPATIAL_PIXELS; i++) {
        LedBlend b = self->blends[i];
        for (unsigned c = 0; c < 3; c++) {
            unsigned val = ((unsigned)rgb[b.a][c] * (65535U - b.weight) +
                            (unsigned)rgb[b.b][c] * b.weight + 32767U) / 65535U;
            self->rgb_buffer[i * 3 + c] = (uint8_t)val;
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
