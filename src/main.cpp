// src/main.cpp — Raspberry Pi SPI SysID CSV capture + auto plot
//
// RPi is SPI master.
// STM32F411 SPI2 is slave.
// Reads 32-byte SysIdSample frames from /dev/spidev0.0.
//
// CS is GPIO25, driven manually via lgpio (active low).
//
// Vector buffering:
//   All frames are collected into a std::vector during capture.
//   CSV is written at the end in one pass.
//   This maximizes SPI throughput and gets closer to 20kHz sample rate.
//
// Run directory:
//   /home/jz/trajectory-streamer/build
//
// CSV output:
//   /home/jz/trajectory-streamer/drive_data/sysid_log.csv
//
// Plot script:
//   /home/jz/trajectory-streamer/py-script/plot_sysid.py

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

#define SYSID_FRAME_LEN  32

#define DEFAULT_DEV      "/dev/spidev0.0"
#define DEFAULT_SPEED_HZ 4000000u        /* 4 MHz — maximizes throughput */

#define DEFAULT_OUTDIR   "../drive_data"
#define DEFAULT_OUTFILE  "../drive_data/sysid_log.csv"
#define DEFAULT_PLOT_SCRIPT "../py-script/plot_sysid.py"

#define CAPTURE_SECONDS  2.0

/* GPIO assignments */
//#define CS_GPIO           25   /* PB12 on STM, active low */
#define READY_REFILL_GPIO  7   /* PC13 on STM, active low */

static volatile sig_atomic_t g_run = 1;

struct __attribute__((packed)) SysIdSample
{
    uint32_t t;
    int16_t  ia_mA;
    int16_t  ib_mA;
    int16_t  ic_mA;
    int16_t  id_mA;
    int16_t  iq_mA;
    int16_t  vd_mV;
    int16_t  vq_mV;
    int16_t  theta_mrad;
    uint16_t adc_a;
    uint16_t adc_b;
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
    uint32_t frames = 0;
    uint32_t first_t = 0;
    uint32_t last_t = 0;
    bool have_t = false;
    uint32_t min_dt = 0xFFFFFFFFu;
    uint32_t max_dt = 0;
    uint64_t sum_dt = 0;
    uint32_t dt_count = 0;
    uint32_t zero_dt_count = 0;
    uint32_t big_dt_count = 0;
    uint32_t torn_frames = 0;
};

static void sigint_handler(int sig)
{
    (void)sig;
    g_run = 0;
}

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
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8)  |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

static SysIdSample decode_sysid_sample(const uint8_t rx[SYSID_FRAME_LEN])
{
    SysIdSample s{};
    s.t          = get_u32_le(&rx[0]);
    s.ia_mA      = get_s16_le(&rx[4]);
    s.ib_mA      = get_s16_le(&rx[6]);
    s.ic_mA      = get_s16_le(&rx[8]);
    s.id_mA      = get_s16_le(&rx[10]);
    s.iq_mA      = get_s16_le(&rx[12]);
    s.vd_mV      = get_s16_le(&rx[14]);
    s.vq_mV      = get_s16_le(&rx[16]);
    s.theta_mrad = get_s16_le(&rx[18]);
    s.adc_a      = get_u16_le(&rx[20]);
    s.adc_b      = get_u16_le(&rx[22]);
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

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0)
    {
        std::perror("SPI_IOC_WR_MODE");
        close(fd);
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
    {
        std::perror("SPI_IOC_WR_BITS_PER_WORD");
        close(fd);
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0)
    {
        std::perror("SPI_IOC_WR_MAX_SPEED_HZ");
        close(fd);
        return -1;
    }

    return fd;
}

static int spi_read_frame(int fd, uint32_t speed_hz,
                          uint8_t rx[SYSID_FRAME_LEN], int gpio_h)
{
    uint8_t tx[SYSID_FRAME_LEN]{};
    std::memset(rx, 0, SYSID_FRAME_LEN);

    //lgGpioWrite(gpio_h, CS_GPIO, 0);

    spi_ioc_transfer tr{};
    tr.tx_buf        = reinterpret_cast<unsigned long>(tx);
    tr.rx_buf        = reinterpret_cast<unsigned long>(rx);
    tr.len           = SYSID_FRAME_LEN;
    tr.speed_hz      = speed_hz;
    tr.bits_per_word = 8;
    tr.delay_usecs   = 0;

    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

   // lgGpioWrite(gpio_h, CS_GPIO, 1);

    if (ret < 1)
    {
        std::perror("SPI_IOC_MESSAGE");
        return -1;
    }

    return 0;
}

static void write_csv_header(FILE *f)
{
    std::fprintf(f,
        "host_time_s,frame,t,dt,ia_mA,ib_mA,ic_mA,"
        "id_mA,iq_mA,vd_mV,vq_mV,theta_mrad,"
        "adc_a,adc_b,adc_c,flags,crc,pad\n");
}

static void write_csv_sample(FILE *f, double host_time_s, uint32_t frame,
                             const SysIdSample *s, uint32_t dt)
{
    std::fprintf(f,
        "%.9f,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,0x%04X,%u,%u\n",
        host_time_s, frame,
        s->t, dt,
        s->ia_mA, s->ib_mA, s->ic_mA,
        s->id_mA, s->iq_mA,
        s->vd_mV, s->vq_mV,
        s->theta_mrad,
        s->adc_a, s->adc_b, s->adc_c,
        s->flags, s->crc, s->pad);
}

static void update_stats(CaptureStats *stats, const SysIdSample *s, uint32_t dt)
{
    if (!stats->have_t)
    {
        stats->first_t = s->t;
        stats->last_t  = s->t;
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
    const double host_frame_rate =
        (elapsed_s > 0.0) ? static_cast<double>(stats->frames) / elapsed_s : 0.0;
    const uint32_t sample_delta =
        stats->have_t ? (stats->last_t - stats->first_t) : 0;
    const double estimated_stm_rate =
        (elapsed_s > 0.0) ? static_cast<double>(sample_delta) / elapsed_s : 0.0;
    const double avg_dt =
        (stats->dt_count > 0)
            ? static_cast<double>(stats->sum_dt) / static_cast<double>(stats->dt_count)
            : 0.0;

    std::printf("\nCapture summary\n");
    std::printf("---------------\n");
    std::printf("File                 : %s\n", out_path);
    std::printf("Elapsed              : %.6f s\n", elapsed_s);
    std::printf("Frames captured      : %u\n", stats->frames);
    std::printf("Host SPI frame rate  : %.1f frames/s\n", host_frame_rate);
    std::printf("First STM t          : %u\n", stats->first_t);
    std::printf("Last STM t           : %u\n", stats->last_t);
    std::printf("STM sample delta     : %u\n", sample_delta);
    std::printf("Estimated STM rate   : %.1f samples/s\n", estimated_stm_rate);
    std::printf("dt min/avg/max       : %u / %.2f / %u\n",
                stats->min_dt == 0xFFFFFFFFu ? 0 : stats->min_dt,
                avg_dt, stats->max_dt);
    std::printf("dt == 0 count        : %u\n", stats->zero_dt_count);
    std::printf("dt > 100 count       : %u\n", stats->big_dt_count);
    std::printf("Torn frames          : %u (%.1f%%)\n",
    stats->torn_frames,
    100.0 * stats->torn_frames / (stats->frames + stats->torn_frames));
}

static int run_plot_script(const char *csv_path)
{
    const char *plot_script = DEFAULT_PLOT_SCRIPT;
    char cwd[PATH_MAX];

    std::printf("\nPlot setup\n----------\n");
    if (getcwd(cwd, sizeof(cwd)) != nullptr)
        std::printf("cwd         : %s\n", cwd);
    else
        std::perror("getcwd");

    std::printf("csv path    : %s\n", csv_path);
    std::printf("plot script : %s\n", plot_script);

    if (access(csv_path, R_OK) != 0)
    {
        std::fprintf(stderr, "CSV not readable: %s\n", csv_path);
        return -1;
    }

    if (access(plot_script, R_OK) != 0)
    {
        std::fprintf(stderr, "Plot script not readable: %s\n", plot_script);
        return -1;
    }

    char cmd[1024];
    std::snprintf(cmd, sizeof(cmd),
        "python3 -u \"%s\" \"%s\"", plot_script, csv_path);

    std::printf("\nRunning plot script\n-------------------\n%s\n", cmd);

    int ret = std::system(cmd);
    if (ret != 0)
    {
        std::fprintf(stderr, "plot_sysid.py failed with code: %d\n", ret);
        return -1;
    }

    std::printf("\nPlot script completed.\n");
    return 0;
}



int main(int argc, char **argv)
{
    const char *dev   = DEFAULT_DEV;
    uint32_t speed_hz = DEFAULT_SPEED_HZ;
    const char *out_path = DEFAULT_OUTFILE;

    if (argc >= 2) dev       = argv[1];
    if (argc >= 3) speed_hz  = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
    if (argc >= 4) out_path  = argv[3];

    std::signal(SIGINT, sigint_handler);

    if (mkdir_if_needed(DEFAULT_OUTDIR) != 0) return 1;

    int gpio_h = lgGpiochipOpen(0);
    if (gpio_h < 0)
    {
        std::fprintf(stderr, "lgGpiochipOpen failed: %d\n", gpio_h);
        return 1;
    }

    // if (lgGpioClaimOutput(gpio_h, 0, CS_GPIO, 1) < 0)
    // {
    //     std::fprintf(stderr, "lgGpioClaimOutput CS_GPIO failed\n");
    //     lgGpiochipClose(gpio_h);
    //     return 1;
    // }

    std::printf("SPI device  : %s\n", dev);
    std::printf("SPI speed   : %u Hz\n", speed_hz);
    //std::printf("CS GPIO     : %d\n", CS_GPIO);
    std::printf("Output      : %s\n", out_path);
    std::printf("Capture     : %.3f seconds\n", CAPTURE_SECONDS);

    int fd = spi_open_configure(dev, speed_hz);
    if (fd < 0) { lgGpiochipClose(gpio_h); return 1; }

    /*
     * Reserve vector capacity upfront.
     * At 20kHz for CAPTURE_SECONDS, worst case ~40k frames.
     * Pre-allocate to avoid reallocation during capture.
     */
    std::vector<CapturedFrame> frames;
    frames.reserve(static_cast<size_t>(CAPTURE_SECONDS * 25000.0));

    CaptureStats stats{};
    uint32_t last_t  = 0;
    bool have_last_t = false;
    const double t0  = monotonic_seconds();

    std::printf("Capturing...\n");

    while (g_run)
    {
        const double now     = monotonic_seconds();
        const double elapsed = now - t0;

        if (elapsed >= CAPTURE_SECONDS) break;

        uint8_t rx[SYSID_FRAME_LEN];

        if (spi_read_frame(fd, speed_hz, rx, gpio_h) < 0)
        {
            close(fd);
            lgGpiochipClose(gpio_h);
            return 1;
        }

        SysIdSample s = decode_sysid_sample(rx);

        if (s.ia_mA + s.ib_mA + s.ic_mA != 0)
        {
            stats.torn_frames++;
            continue;
        }   

        uint32_t dt = 0;
        if (have_last_t) dt = s.t - last_t;
        last_t       = s.t;
        have_last_t  = true;

        CapturedFrame cf;
        cf.host_time_s = elapsed;
        cf.sample      = s;
        cf.dt          = dt;
        frames.push_back(cf);

        update_stats(&stats, &s, dt);
        stats.frames++;
    }

    const double elapsed_total = monotonic_seconds() - t0;

    close(fd);
    lgGpiochipClose(gpio_h);

    std::printf("Capture complete. Writing %zu frames to CSV...\n", frames.size());

    /*
     * Write CSV after capture — no disk I/O during hot loop.
     */
    FILE *f = std::fopen(out_path, "w");
    if (!f)
    {
        std::fprintf(stderr, "Failed to open output file: %s\n", out_path);
        return 1;
    }

    write_csv_header(f);

    for (uint32_t i = 0; i < static_cast<uint32_t>(frames.size()); i++)
    {
        write_csv_sample(f, frames[i].host_time_s, i,
                         &frames[i].sample, frames[i].dt);
    }

    std::fflush(f);
    std::fclose(f);

    print_summary(&stats, elapsed_total, out_path);

    if (run_plot_script(out_path) != 0) return 1;

    return 0;
}