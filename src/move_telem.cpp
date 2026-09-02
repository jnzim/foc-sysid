// move_telem.cpp — capture one boot-triggered move's real TelemetryFrame
// telemetry, write it to CSV, plot it.
//
// This is a separate, minimal tool -- it does not touch main.cpp or any of
// the existing SysID test types. It reads the STM's TelemetryFrame wire
// format directly (protocol.h), field for field, with no relabeling: the
// CSV columns are exactly the struct's own field names.
//
// At this step the STM starts its move on its own at boot (no MoveCmd sent
// yet -- that's a later step); this tool just pulses the same boot-gate
// GPIO trigger the existing main.cpp tool uses, then captures whatever
// telemetry comes back until the move is done.
//
// "Done" = drive_state was DRIVE_SERVO_ON at some point, then went back to
// DRIVE_IDLE. No magic numbers, no test_id markers -- just the real state.

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <lgpio.h>

#include "protocol.h"

#define FRAME_LEN            32
#define DEFAULT_DEV          "/dev/spidev0.0"
#define DEFAULT_SPEED_HZ     4000000u
#define TARGET_FRAME_RATE_HZ 10000.0
#define TIMEOUT_SECONDS      15.0     // ALIGN (5s) + move + scaffolding hold, plus margin
#define PIN_FIRE_SYSID       3        // Pi GPIO3 -> STM PC3, active-low boot-gate trigger
#define OUT_CSV              "../drive_data/latest/move_telem.csv"
#define PLOT_SCRIPT          "../py-script/plot_move.py"

static int spi_open_configure(const char *dev, uint32_t speed_hz)
{
    int fd = open(dev, O_RDWR);
    if (fd < 0) { perror("open SPI device"); return -1; }

    uint8_t mode = SPI_MODE_1;
    uint8_t bits = 8;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0)
    {
        perror("SPI ioctl");
        close(fd);
        return -1;
    }
    return fd;
}

static int spi_read_frame(int fd, uint32_t speed_hz, uint8_t rx[FRAME_LEN])
{
    uint8_t tx[FRAME_LEN] = {0};
    memset(rx, 0, FRAME_LEN);

    spi_ioc_transfer tr{};
    tr.tx_buf        = reinterpret_cast<unsigned long>(tx);
    tr.rx_buf        = reinterpret_cast<unsigned long>(rx);
    tr.len           = FRAME_LEN;
    tr.speed_hz      = speed_hz;
    tr.bits_per_word = 8;

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1)
    {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }
    return 0;
}

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

int main(int argc, char **argv)
{
    const char *dev      = (argc > 1) ? argv[1] : DEFAULT_DEV;
    uint32_t    speed_hz = (argc > 2) ? static_cast<uint32_t>(strtoul(argv[2], nullptr, 0))
                                       : DEFAULT_SPEED_HZ;

    static_assert(sizeof(TelemetryFrame) == FRAME_LEN, "TelemetryFrame must be 32 bytes");

    int gpio_h = lgGpiochipOpen(0);
    if (gpio_h < 0) { fprintf(stderr, "lgGpiochipOpen failed\n"); return 1; }

    lgGpioFree(gpio_h, PIN_FIRE_SYSID);
    if (lgGpioClaimOutput(gpio_h, 0, PIN_FIRE_SYSID, 1) < 0)
    {
        fprintf(stderr, "lgGpioClaimOutput failed\n");
        lgGpiochipClose(gpio_h);
        return 1;
    }

    int fd = spi_open_configure(dev, speed_hz);
    if (fd < 0) { lgGpiochipClose(gpio_h); return 1; }

    printf("device : %s\nspeed  : %u Hz\ntimeout: %.1f s\n", dev, speed_hz, TIMEOUT_SECONDS);

    // Boot-gate trigger -- same 10ms active-low pulse main.cpp uses. The
    // STM blocks at boot until it sees this, regardless of what firmware
    // mode it's running.
    lgGpioWrite(gpio_h, PIN_FIRE_SYSID, 0);
    usleep(10000);
    lgGpioWrite(gpio_h, PIN_FIRE_SYSID, 1);
    printf("trigger sent, capturing...\n");

    struct Sample { double host_time_s; TelemetryFrame frame; };
    std::vector<Sample> samples;
    samples.reserve(20000);

    uint32_t torn_frames = 0;
    bool     saw_servo_on = false;

    const double t0 = now_s();
    const double frame_period_s = 1.0 / TARGET_FRAME_RATE_HZ;
    double next_frame_time = 0.0;

    while (true)
    {
        double elapsed = now_s() - t0;
        if (elapsed > TIMEOUT_SECONDS)
        {
            fprintf(stderr, "timeout waiting for SERVO_ON -> IDLE\n");
            break;
        }

        double wait_s = next_frame_time - elapsed;
        if (wait_s > 0.0) usleep(static_cast<useconds_t>(wait_s * 1e6));
        next_frame_time += frame_period_s;

        uint8_t rx[FRAME_LEN];
        if (spi_read_frame(fd, speed_hz, rx) < 0) break;

        TelemetryFrame frame;
        memcpy(&frame, rx, FRAME_LEN);

        uint16_t calc = crc16_calc(rx, TELEM_CRC_LEN);
        if (calc != frame.crc16) { torn_frames++; continue; }

        samples.push_back({now_s() - t0, frame});

        if (frame.drive_state == DRIVE_SERVO_ON) saw_servo_on = true;
        if (saw_servo_on && frame.drive_state == DRIVE_IDLE)
        {
            printf("move complete after %zu frames (%u torn)\n", samples.size(), torn_frames);
            break;
        }
    }

    close(fd);
    lgGpiochipClose(gpio_h);

    if (samples.empty())
    {
        fprintf(stderr, "no valid frames captured\n");
        return 1;
    }

    FILE *csv = fopen(OUT_CSV, "w");
    if (!csv) { perror("fopen csv"); return 1; }

    fprintf(csv, "host_time_s,pos_cmd,pos_fbk,vel_cmd,vel_fbk,timestamp_ms,"
                 "drive_state,fault_flags,samples_consumed,iq_cmd,i_q_fbk\n");
    for (const auto &s : samples)
    {
        const TelemetryFrame &f = s.frame;
        fprintf(csv, "%.9f,%d,%d,%d,%d,%u,%u,%u,%u,%d,%d\n",
                s.host_time_s, f.pos_cmd, f.pos_fbk, f.vel_cmd, f.vel_fbk,
                f.timestamp_ms, f.drive_state, f.fault_flags, f.samples_consumed,
                f.iq_cmd, f.i_q_fbk);
    }
    fclose(csv);
    printf("wrote %s\n", OUT_CSV);

    int rc = system("python3 " PLOT_SCRIPT);
    (void)rc;

    return 0;
}
