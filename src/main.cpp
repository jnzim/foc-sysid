// src/main.cpp — Raspberry Pi SPI capture + Bode analysis trigger
//
// Usage:
//   ./drive [spi_dev [speed_hz [out_csv]]]
//
// CSV columns:
//   host_time_s, frame, t, dt, encoder_position,
//   sysid_f, id_mA, iq_mA, vd_mV, vq_mV, theta_mrad,
//   ia_mA, ib_mA, iq_cmd_mA, flags, crc, pad,
//   enc_hi_raw, enc_lo_raw

#include <cerrno>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <limits.h>
#include <vector>
#include <map>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <lgpio.h>

#include "protocol.h"   // crc16_calc() -- same CCITT CRC the STM firmware now stamps into every frame

#define SYSID_FRAME_LEN     32
#define DEFAULT_DEV         "/dev/spidev0.0"
#define DEFAULT_SPEED_HZ    4000000u
// Read loop was issuing back-to-back SPI transactions as fast as the bus
// and kernel driver allowed (no pacing at all) -- at 4MHz/32B that's on the
// order of 10+ kHz of CS toggles, each one firing the STM's highest-priority
// IRQ (EXTI15_10, NVIC prio 0). That was starving the ADC's own interrupt
// (prio 3) for multiple ms at a time during CL_VEL_CHIRP instability events
// (confirmed via ADC1->SR telemetry -- JEOC set and never serviced). Current
// loop ID is done, so we don't need 10kHz+ telemetry -- throttling the host
// read rate down to 5kHz gives the STM's IRQ priorities enough breathing
// room between transactions.
#define TARGET_FRAME_RATE_HZ  10000.0
// "latest" is overwritten every run — no digging through old plots to find
// the one from the test you just ran. Move out anything you want to keep.
#define DEFAULT_OUTDIR      "../drive_data/latest"
#define DEFAULT_OUTFILE     "../drive_data/latest/sysid_log.csv"
#define CAPTURE_TIMEOUT_SECONDS  120.0
#define SYSID_STAGE_RUN          1u
#define SYSID_STAGE_IDLE         2u
#define PIN_FIRE_SYSID      3    // Pi GPIO3 → STM PC3, active-low trigger

// Mirrors Include/config.h SYSID_TEST_* values on the STM32 side.
// The firmware stamps its compile-time SYSID_TEST into SysIdSample.pad
// (see spi_sysid_update_latest() in src/spi.c) so this side always knows
// which test actually ran, instead of guessing which script to run.
#define SYSID_TEST_CURRENT_LOOP_CHIRP  0u
#define SYSID_TEST_CURRENT_LOOP_STEP   1u
#define SYSID_TEST_VEL_CHIRP           2u
#define SYSID_TEST_CL_VEL_STEP         4u
#define SYSID_TEST_RIPPLE_DEBUG        5u
#define SYSID_TEST_CL_VEL_CHIRP        6u
#define SYSID_TEST_POSITION_STEP       7u
#define SYSID_TEST_CL_POS_CHIRP        8u
#define SYSID_TEST_CINE_SWEEP          9u

static volatile sig_atomic_t g_run = 1;

struct __attribute__((packed)) SysIdSample
{
    uint32_t t;
    int16_t  enc_hi;
    int16_t  enc_lo;
    int16_t  sysid_f;
    int16_t  id_mA;
    int16_t  iq_mA;
    int16_t  vd_mV;
    int16_t  vq_mV;
    int16_t  theta_mrad;
    int16_t  ia_mA;
    int16_t  ib_mA;
    int16_t  iq_cmd_mA;
    uint16_t flags;
    uint16_t crc;
    uint16_t pad;
};

static_assert(sizeof(SysIdSample) == 32);

struct CapturedFrame
{
    double      host_time_s;
    SysIdSample sample;
    uint32_t    dt;
};

struct CaptureStats
{
    uint32_t frames        = 0;
    uint32_t first_t       = 0;
    uint32_t last_t        = 0;
    bool     have_t        = false;
    uint32_t min_dt        = 0xFFFFFFFFu;
    uint32_t max_dt        = 0;
    uint64_t sum_dt        = 0;
    uint32_t dt_count      = 0;
    uint32_t zero_dt_count = 0;
    uint32_t big_dt_count  = 0;
    uint32_t torn_frames   = 0;
};

static void sigint_handler(int sig) { (void)sig; g_run = 0; }

static double monotonic_seconds()
{
    timespec ts{};
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
        std::perror("clock_gettime");
        return 0.0;
    }
    return static_cast<double>(ts.tv_sec) +
           static_cast<double>(ts.tv_nsec) * 1.0e-9;
}

static uint16_t get_u16_le(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8);
}

static int16_t get_s16_le(const uint8_t *p)
{
    return static_cast<int16_t>(get_u16_le(p));
}

static uint32_t get_u32_le(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0])        |
           (static_cast<uint32_t>(p[1]) << 8)  |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

static SysIdSample decode_sysid_sample(const uint8_t rx[SYSID_FRAME_LEN])
{
    SysIdSample s{};
    s.t          = get_u32_le(&rx[0]);
    s.enc_hi     = get_s16_le(&rx[4]);
    s.enc_lo     = get_s16_le(&rx[6]);
    s.sysid_f    = get_s16_le(&rx[8]);
    s.id_mA      = get_s16_le(&rx[10]);
    s.iq_mA      = get_s16_le(&rx[12]);
    s.vd_mV      = get_s16_le(&rx[14]);
    s.vq_mV      = get_s16_le(&rx[16]);
    s.theta_mrad = get_s16_le(&rx[18]);
    s.ia_mA      = get_s16_le(&rx[20]);
    s.ib_mA      = get_s16_le(&rx[22]);
    s.iq_cmd_mA  = get_s16_le(&rx[24]);
    s.flags      = get_u16_le(&rx[26]);
    s.crc        = get_u16_le(&rx[28]);
    s.pad        = get_u16_le(&rx[30]);
    return s;
}

static int mkdir_if_needed(const char *path)
{
    struct stat st{};
    if (stat(path, &st) == 0)
    {
        if (S_ISDIR(st.st_mode)) return 0;
        std::fprintf(stderr, "Path exists but is not a directory: %s\n", path);
        return -1;
    }
    if (mkdir(path, 0775) != 0)
    {
        if (errno == EEXIST) return 0;
        std::perror("mkdir");
        return -1;
    }
    return 0;
}

static int spi_open_configure(const char *dev, uint32_t speed_hz)
{
    int fd = open(dev, O_RDWR);
    if (fd < 0) { std::perror("open SPI device"); return -1; }

    uint8_t mode = SPI_MODE_1;
    uint8_t bits = 8;

    if (ioctl(fd, SPI_IOC_WR_MODE,         &mode)     < 0 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)     < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ,  &speed_hz) < 0)
    {
        std::perror("SPI ioctl");
        close(fd);
        return -1;
    }
    return fd;
}

static int spi_read_frame(int fd, uint32_t speed_hz, uint8_t rx[SYSID_FRAME_LEN])
{
    uint8_t tx[SYSID_FRAME_LEN]{};
    std::memset(rx, 0, SYSID_FRAME_LEN);

    spi_ioc_transfer tr{};
    tr.tx_buf        = reinterpret_cast<unsigned long>(tx);
    tr.rx_buf        = reinterpret_cast<unsigned long>(rx);
    tr.len           = SYSID_FRAME_LEN;
    tr.speed_hz      = speed_hz;
    tr.bits_per_word = 8;
    tr.delay_usecs   = 0;

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1)
    {
        std::perror("SPI_IOC_MESSAGE");
        return -1;
    }
    return 0;
}

// static void write_csv_header(FILE *f)
// {
//     std::fprintf(f,
//         "host_time_s,frame,t,dt,"
//         "enc_hi,enc_lo,sysid_f,"
//         "id_mA,iq_mA,vd_mV,vq_mV,theta_mrad,"
//         "ia_mA,ib_mA,adc_c,flags,crc,pad\n");
// }

// encoder_position is enc_hi/enc_lo combined into a real 32-bit encoder
// count -- what most tests actually want. enc_hi_raw/enc_lo_raw pass the two
// halves through unmodified: the position-loop tests (SYSID_TEST_POSITION_STEP,
// SYSID_TEST_CL_POS_CHIRP) repurpose them on the STM32 side to carry
// vel_cmd_rad_sec [mrad/s] and measured velocity [counts/s] instead, since
// encoder_position is otherwise fully redundant with pos_meas for those tests
// (see foc_sysid_step() telemetry section in foc_sysid.c).
static void write_csv_header(FILE *f)
{
    std::fprintf(
        f,
        "host_time_s,frame,t,dt,encoder_position,"
        "sysid_f,id_mA,iq_mA,vd_mV,vq_mV,theta_mrad,"
        "ia_mA,ib_mA,iq_cmd_mA,flags,crc,pad,enc_hi_raw,enc_lo_raw\n"
    );
}
static void write_csv_sample(FILE *f, double host_time_s, uint32_t frame,
                             const SysIdSample *s, uint32_t dt)
{
    const uint32_t encoder_bits =
        (static_cast<uint32_t>(static_cast<uint16_t>(s->enc_hi)) << 16) |
         static_cast<uint32_t>(static_cast<uint16_t>(s->enc_lo));

    const int32_t encoder_position =
        static_cast<int32_t>(encoder_bits);

    std::fprintf(
        f,
        "%.9f,%u,%u,%u,"
        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
        "0x%04X,%u,%u,%d,%d\n",
        host_time_s,
        frame,
        s->t,
        dt,
        encoder_position,
        static_cast<int>(s->sysid_f),
        static_cast<int>(s->id_mA),
        static_cast<int>(s->iq_mA),
        static_cast<int>(s->vd_mV),
        static_cast<int>(s->vq_mV),
        static_cast<int>(s->theta_mrad),
        static_cast<int>(s->ia_mA),
        static_cast<int>(s->ib_mA),
        static_cast<int>(s->iq_cmd_mA),
        static_cast<unsigned>(s->flags),
        static_cast<unsigned>(s->crc),
        static_cast<unsigned>(s->pad),
        static_cast<int>(s->enc_hi),
        static_cast<int>(s->enc_lo)
    );
}

static void update_stats(CaptureStats *stats, const SysIdSample *s, uint32_t dt)
{
    if (!stats->have_t)
    {
        stats->first_t = stats->last_t = s->t;
        stats->have_t  = true;
        return;
    }
    stats->last_t = s->t;
    stats->sum_dt += dt;
    stats->dt_count++;
    if (dt < stats->min_dt) stats->min_dt = dt;
    if (dt > stats->max_dt) stats->max_dt = dt;
    if (dt == 0)   stats->zero_dt_count++;
    if (dt > 100)  stats->big_dt_count++;
}

static void print_summary(const CaptureStats *stats, double elapsed_s,
                          const char *out_path)
{
    const double   host_fps    = (elapsed_s > 0.0) ? stats->frames / elapsed_s : 0.0;
    const uint32_t sample_span = stats->have_t ? (stats->last_t - stats->first_t) : 0;
    const double   stm_rate    = (elapsed_s > 0.0) ? sample_span / elapsed_s : 0.0;
    const double   avg_dt      = stats->dt_count > 0
                                    ? (double)stats->sum_dt / stats->dt_count : 0.0;
    const uint32_t total       = stats->frames + stats->torn_frames;

    std::printf("\nCapture summary\n---------------\n");
    std::printf("File                 : %s\n",   out_path);
    std::printf("Elapsed              : %.6f s\n", elapsed_s);
    std::printf("Frames captured      : %u\n",   stats->frames);
    std::printf("Host SPI frame rate  : %.1f frames/s\n", host_fps);
    std::printf("STM sample delta     : %u\n",   sample_span);
    std::printf("Estimated STM rate   : %.1f samples/s\n", stm_rate);
    std::printf("dt min/avg/max       : %u / %.2f / %u\n",
                stats->min_dt == 0xFFFFFFFFu ? 0 : stats->min_dt,
                avg_dt, stats->max_dt);
    std::printf("dt == 0 count        : %u\n",   stats->zero_dt_count);
    std::printf("dt > 100 count       : %u\n",   stats->big_dt_count);
    std::printf("Torn frames          : %u (%.1f%%)\n",
                stats->torn_frames,
                total > 0 ? 100.0 * stats->torn_frames / total : 0.0);
}

// Which analysis script(s) to run for a given SYSID_TEST, read back from
// SysIdSample.pad. Some tests have a companion time-domain plot alongside
// the primary Bode/step analysis.
static std::vector<const char *> plot_scripts_for_test(uint16_t test_id)
{
    switch (test_id)
    {
        case SYSID_TEST_CURRENT_LOOP_CHIRP:
            return { "../py-script/bode_plot.py" };
        case SYSID_TEST_CURRENT_LOOP_STEP:
            return { "../py-script/step.py" };
        case SYSID_TEST_VEL_CHIRP:
            return { "../py-script/bode_vel_plot.py", "../py-script/vel_v_t.py" };
        case SYSID_TEST_CL_VEL_STEP:
            return { "../py-script/velocity_step_plot.py" };
        case SYSID_TEST_RIPPLE_DEBUG:
            return { "../py-script/ripple_debug_plot.py" };
        case SYSID_TEST_CL_VEL_CHIRP:
            return { "../py-script/closed_vel_bode_plot.py" };
        case SYSID_TEST_POSITION_STEP:
            return { "../py-script/position_step_plot.py" };
        case SYSID_TEST_CL_POS_CHIRP:
            return { "../py-script/closed_pos_bode_plot.py" };
        case SYSID_TEST_CINE_SWEEP:
            return { "../py-script/cine_overlay_render.py" };
        default:
            return {};
    }
}

static int run_plot_scripts(const char *csv_path, uint16_t test_id)
{
    char cwd[PATH_MAX];

    std::printf("\nPlot setup\n----------\n");
    if (getcwd(cwd, sizeof(cwd))) std::printf("cwd     : %s\n", cwd);
    std::printf("csv     : %s\n", csv_path);
    std::printf("test_id : %u\n", test_id);

    if (access(csv_path, R_OK) != 0) { std::perror("csv not readable"); return -1; }

    std::vector<const char *> scripts = plot_scripts_for_test(test_id);
    if (scripts.empty())
    {
        std::fprintf(stderr,
                      "no analysis script mapped for SYSID_TEST=%u — run one manually\n",
                      test_id);
        return -1;
    }

    int rc = 0;
    for (const char *script : scripts)
    {
        std::printf("script  : %s\n", script);

        if (access(script, R_OK) != 0)
        {
            std::perror("script not readable");
            rc = -1;
            continue;
        }

        char cmd[1024];
        std::snprintf(cmd, sizeof(cmd), "python3 -u \"%s\" \"%s\"", script, csv_path);
        std::printf("\n%s\n", cmd);

        int ret = std::system(cmd);
        if (ret != 0)
        {
            std::fprintf(stderr, "%s exited with code %d\n", script, ret);
            rc = -1;
        }
    }
    return rc;
}

// Pops open every .png in `dir` newer than `since` on the Pi's local
// monitor (DISPLAY=:0 — the physically attached display, not this SSH
// session). Best-effort: failures are logged, never fatal.
static void open_new_plots(const char *dir, time_t since)
{
    DIR *d = opendir(dir);
    if (!d) { std::perror("opendir plot dir"); return; }

    struct dirent *entry;
    while ((entry = readdir(d)) != nullptr)
    {
        const char *name = entry->d_name;
        size_t      len  = std::strlen(name);
        if (len < 4 || std::strcmp(name + len - 4, ".png") != 0)
            continue;

        char path[PATH_MAX];
        std::snprintf(path, sizeof(path), "%s/%s", dir, name);

        struct stat st{};
        if (stat(path, &st) != 0 || st.st_mtime < since)
            continue;

        std::printf("opening : %s\n", path);

        char cmd[PATH_MAX + 64];
        std::snprintf(cmd, sizeof(cmd),
                      "DISPLAY=:0 xdg-open \"%s\" >/dev/null 2>&1 &", path);
        std::system(cmd);
    }
    closedir(d);
}

// Blocks on a line of stdin. Returns false on EOF/Ctrl-D (or a signal
// interrupting the read), which the caller treats as "stop looping".
static bool wait_for_enter(const char *prompt)
{
    std::printf("\n%s", prompt);
    std::fflush(stdout);

    char line[64];
    return std::fgets(line, sizeof(line), stdin) != nullptr;
}

int main(int argc, char **argv)
{
    const char *dev      = DEFAULT_DEV;
    uint32_t    speed_hz = DEFAULT_SPEED_HZ;
    const char *out_path = DEFAULT_OUTFILE;

    if (argc >= 2) dev      = argv[1];
    if (argc >= 3) speed_hz = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
    if (argc >= 4) out_path = argv[3];

    std::signal(SIGINT, sigint_handler);

    if (mkdir_if_needed(DEFAULT_OUTDIR) != 0) return 1;

    int gpio_h = lgGpiochipOpen(0);
    if (gpio_h < 0) { std::fprintf(stderr, "lgGpiochipOpen failed\n"); return 1; }

    lgGpioFree(gpio_h, PIN_FIRE_SYSID);
    if (lgGpioClaimOutput(gpio_h, 0, PIN_FIRE_SYSID, 1) < 0)
    {
        std::fprintf(stderr, "lgGpioClaimOutput failed\n");
        lgGpiochipClose(gpio_h);
        return 1;
    }

    std::printf("device  : %s\n",   dev);
    std::printf("speed   : %u Hz\n", speed_hz);
    std::printf("output  : %s\n",   out_path);
    std::printf("timeout : %.1f s\n", CAPTURE_TIMEOUT_SECONDS);

    int fd = spi_open_configure(dev, speed_hz);
    if (fd < 0) { lgGpiochipClose(gpio_h); return 1; }

    int exit_code = 0;

    // Re-arms after every capture instead of exiting — flash/reset the STM
    // for the next test, then hit Enter here when it's booted and ready.
    while (g_run)
    {
        if (!wait_for_enter("Press Enter once the STM is flashed and ready "
                            "for the next test (Ctrl+C to exit): "))
            break;

        if (!g_run) break;

        // Trigger STM — 10ms active-low pulse on PIN_FIRE_SYSID
        lgGpioWrite(gpio_h, PIN_FIRE_SYSID, 0);
        usleep(10000);
        lgGpioWrite(gpio_h, PIN_FIRE_SYSID, 1);
        std::printf("trigger sent, capturing...\n");

        std::vector<CapturedFrame> frames;
        frames.reserve(static_cast<size_t>(CAPTURE_TIMEOUT_SECONDS * 12000.0));

        CaptureStats stats{};
        uint32_t last_t    = 0;
        bool     have_last = false;
        bool     saw_run   = false;
        bool     read_error = false;
        const double t0    = monotonic_seconds();
        const double frame_period_s = 1.0 / TARGET_FRAME_RATE_HZ;
        double next_frame_time = 0.0;

        while (g_run)
        {
            const double elapsed_now = monotonic_seconds() - t0;

            if (elapsed_now >= CAPTURE_TIMEOUT_SECONDS)
            {
                std::fprintf(stderr,
                             "capture timeout reached before SYSID completed\n");
                break;
            }

            // Pace transactions to TARGET_FRAME_RATE_HZ instead of hammering
            // the bus as fast as it'll go -- fixed schedule (not sleep-after-
            // read) so pacing doesn't drift with read jitter.
            const double wait_s = next_frame_time - elapsed_now;
            if (wait_s > 0.0)
                usleep(static_cast<useconds_t>(wait_s * 1e6));
            next_frame_time += frame_period_s;

            uint8_t rx[SYSID_FRAME_LEN];
            if (spi_read_frame(fd, speed_hz, rx) < 0)
            {
                read_error = true;
                break;
            }

            // CRC check -- this was the actual bug behind "test ends too
            // early": SysIdSample.crc was hardcoded to 0 on the firmware
            // side (no integrity checking at all), so a single corrupted
            // frame's flags byte could alias into a valid-looking
            // SYSID_STAGE_IDLE and make this loop think the sweep finished
            // partway through. Firmware now stamps a real CRC (spi.c); a
            // mismatch here means a torn/corrupted frame -- discard it
            // entirely rather than let it touch dt/stage-detection/output.
            const uint16_t crc_calc = crc16_calc(rx, offsetof(SysIdSample, crc));
            const uint16_t crc_rx   = get_u16_le(&rx[offsetof(SysIdSample, crc)]);
            if (crc_calc != crc_rx)
            {
                stats.torn_frames++;
                continue;
            }

            SysIdSample s = decode_sysid_sample(rx);

            uint32_t dt = have_last ? (s.t - last_t) : 0;
            last_t      = s.t;
            have_last   = true;

            frames.push_back({elapsed_now, s, dt});
            update_stats(&stats, &s, dt);
            stats.frames++;

            const uint16_t stage = s.flags & 0x0003u;

            if (stage == SYSID_STAGE_RUN)
                saw_run = true;

            if (saw_run && stage == SYSID_STAGE_IDLE)
            {
                std::printf("SYSID complete, stopping capture.\n");
                break;
            }
        }

        if (read_error)
        {
            std::fprintf(stderr, "SPI read failed, aborting.\n");
            exit_code = 1;
            break;
        }

        const double elapsed = monotonic_seconds() - t0;

        std::printf("captured %zu frames, writing CSV...\n", frames.size());

        FILE *f = std::fopen(out_path, "w");
        if (!f)
        {
            std::perror("fopen");
            exit_code = 1;
            break;
        }

        write_csv_header(f);
        for (uint32_t i = 0; i < (uint32_t)frames.size(); i++)
            write_csv_sample(f, frames[i].host_time_s, i, &frames[i].sample, frames[i].dt);

        std::fflush(f);
        std::fclose(f);

        print_summary(&stats, elapsed, out_path);

        // pad carries the STM32's compile-time SYSID_TEST value (same on every
        // frame) — pull it from the first RUN-stage frame so a stray boot/idle
        // frame at the front of the capture can't throw it off.
        uint16_t test_id = 0;
        bool     have_test_id = false;
        for (const CapturedFrame &cf : frames)
        {
            if ((cf.sample.flags & 0x0003u) == SYSID_STAGE_RUN)
            {
                test_id      = cf.sample.pad;
                have_test_id = true;
                break;
            }
        }
        if (!have_test_id && !frames.empty())
            test_id = frames.back().sample.pad;

        // A failed plot doesn't end the session — you may still want to
        // reflash and try the next test.
        const time_t plot_start = time(nullptr);
        run_plot_scripts(out_path, test_id);
        open_new_plots(DEFAULT_OUTDIR, plot_start);
    }

    close(fd);
    lgGpiochipClose(gpio_h);
    std::printf("\nExiting.\n");
    return exit_code;
}