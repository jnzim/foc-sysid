// src/main.cpp — Raspberry Pi SPI capture + Bode analysis trigger
//
// Usage:
//   ./drive [spi_dev [speed_hz [out_csv]]]
//
// CSV columns:
//   host_time_s, frame, t, dt,
//   enc_hi, enc_lo, sysid_f,
//   id_mA, iq_mA, vd_mV, vq_mV, theta_mrad,
//   ia_mA, ib_mA, adc_c, flags, crc, pad

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits.h>
#include <vector>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <lgpio.h>

#define SYSID_FRAME_LEN     32
#define DEFAULT_DEV         "/dev/spidev0.0"
#define DEFAULT_SPEED_HZ    4000000u
#define DEFAULT_OUTDIR      "../drive_data"
#define DEFAULT_OUTFILE     "../drive_data/sysid_log.csv"
#define DEFAULT_PLOT_SCRIPT "../py-script/bode_plot.py"
#define CAPTURE_SECONDS     20.0
#define PIN_FIRE_SYSID      3    // Pi GPIO3 → STM PC3, active-low trigger

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
    uint16_t adc_c;
    uint16_t flags;
    uint16_t crc;
    uint16_t pad;
};

static_assert(sizeof(SysIdSample) == 32, "SysIdSample must be exactly 32 bytes");

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
    s.adc_c      = get_u16_le(&rx[24]);
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

static void write_csv_header(FILE *f)
{
    std::fprintf(f,
        "host_time_s,frame,t,dt,"
        "enc_hi,enc_lo,sysid_f,"
        "id_mA,iq_mA,vd_mV,vq_mV,theta_mrad,"
        "ia_mA,ib_mA,adc_c,flags,crc,pad\n");
}

static void write_csv_sample(FILE *f, double host_time_s, uint32_t frame,
                             const SysIdSample *s, uint32_t dt)
{
    std::fprintf(f,
        "%.9f,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,0x%04X,%u,%u\n",
        host_time_s, frame,
        s->t, dt,
        s->enc_hi, s->enc_lo, s->sysid_f,
        s->id_mA, s->iq_mA,
        s->vd_mV, s->vq_mV, s->theta_mrad,
        s->ia_mA, s->ib_mA, s->adc_c,
        s->flags, s->crc, s->pad);
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

static int run_plot_script(const char *csv_path)
{
    const char *script = DEFAULT_PLOT_SCRIPT;
    char cwd[PATH_MAX];

    std::printf("\nPlot setup\n----------\n");
    if (getcwd(cwd, sizeof(cwd))) std::printf("cwd    : %s\n", cwd);
    std::printf("csv    : %s\n", csv_path);
    std::printf("script : %s\n", script);

    if (access(csv_path, R_OK) != 0) { std::perror("csv not readable"); return -1; }
    if (access(script,   R_OK) != 0) { std::perror("script not readable"); return -1; }

    char cmd[1024];
    std::snprintf(cmd, sizeof(cmd), "python3 -u \"%s\" \"%s\"", script, csv_path);
    std::printf("\n%s\n", cmd);

    int ret = std::system(cmd);
    if (ret != 0)
    {
        std::fprintf(stderr, "bode_plot.py exited with code %d\n", ret);
        return -1;
    }
    return 0;
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
    std::printf("capture : %.1f s\n", CAPTURE_SECONDS);

    int fd = spi_open_configure(dev, speed_hz);
    if (fd < 0) { lgGpiochipClose(gpio_h); return 1; }

    // Trigger STM — 10ms active-low pulse on PIN_FIRE_SYSID
    lgGpioWrite(gpio_h, PIN_FIRE_SYSID, 0);
    usleep(10000);
    lgGpioWrite(gpio_h, PIN_FIRE_SYSID, 1);
    std::printf("trigger sent, capturing...\n");

    std::vector<CapturedFrame> frames;
    frames.reserve(static_cast<size_t>(CAPTURE_SECONDS * 25000.0));

    CaptureStats stats{};
    uint32_t last_t    = 0;
    bool     have_last = false;
    const double t0    = monotonic_seconds();

    while (g_run)
    {
        if (monotonic_seconds() - t0 >= CAPTURE_SECONDS) break;

        uint8_t rx[SYSID_FRAME_LEN];
        if (spi_read_frame(fd, speed_hz, rx) < 0)
        {
            close(fd);
            lgGpiochipClose(gpio_h);
            return 1;
        }

        SysIdSample s = decode_sysid_sample(rx);

        uint32_t dt = have_last ? (s.t - last_t) : 0;
        last_t      = s.t;
        have_last   = true;

        frames.push_back({monotonic_seconds() - t0, s, dt});
        update_stats(&stats, &s, dt);
        stats.frames++;
    }

    const double elapsed = monotonic_seconds() - t0;
    close(fd);
    lgGpiochipClose(gpio_h);

    std::printf("captured %zu frames, writing CSV...\n", frames.size());

    FILE *f = std::fopen(out_path, "w");
    if (!f) { std::perror("fopen"); return 1; }

    write_csv_header(f);
    for (uint32_t i = 0; i < (uint32_t)frames.size(); i++)
        write_csv_sample(f, frames[i].host_time_s, i, &frames[i].sample, frames[i].dt);

    std::fflush(f);
    std::fclose(f);

    print_summary(&stats, elapsed, out_path);

    return run_plot_script(out_path) == 0 ? 0 : 1;
}