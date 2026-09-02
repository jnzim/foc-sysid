// move_telem.cpp — send one MoveCmd, capture the real TelemetryFrame
// telemetry for that move, write it to CSV, plot it.
//
// This is a separate, minimal tool -- it does not touch main.cpp or any of
// the existing SysID test types. It reads the STM's TelemetryFrame wire
// format directly (protocol.h), field for field, with no relabeling: the
// CSV columns are exactly the struct's own field names.
//
// Sequence: pulse the boot-gate GPIO trigger (same one main.cpp uses --
// the STM blocks at boot until it sees this, regardless of firmware mode),
// send one MoveCmd as the first SPI transaction, then capture telemetry
// until the move is done. Sending MoveCmd before the STM's ~5s internal
// ALIGN finishes is fine -- foc_trajectory_start_move() (the SPI RX ISR's
// handler) just latches the request; the position tick won't start the
// generator until ALIGN completes. See foc_trajectory.c for why that's safe.
//
// "Done" = drive_state was DRIVE_SERVO_ON at some point, then went back to
// DRIVE_IDLE. No magic numbers, no test_id markers -- just the real state.

#include <cerrno>
#include <csignal>
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

// tx_override, if non-null, rides out on MOSI for this one transaction
// (used to send the MoveCmd on the very first read of a capture) --
// simultaneously reads back whatever telemetry is live on MISO, same
// full-duplex transaction as every other read here.
static int spi_read_frame(int fd, uint32_t speed_hz, uint8_t rx[FRAME_LEN],
                           const uint8_t *tx_override = nullptr)
{
    uint8_t tx[FRAME_LEN] = {0};
    if (tx_override) memcpy(tx, tx_override, FRAME_LEN);
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

// Builds the 32-byte MoveCmd wire frame for one move.
static void build_move_cmd(uint8_t frame[FRAME_LEN], int32_t target_pos,
                            int32_t vel_max, int32_t accel_max)
{
    static_assert(sizeof(MoveCmd) == FRAME_LEN, "MoveCmd must be 32 bytes");

    MoveCmd cmd{};
    cmd.opcode     = SPI2_OP_MOVE_CMD;
    cmd.seq        = 1;
    cmd.target_pos = target_pos;
    cmd.vel_max    = vel_max;
    cmd.accel_max  = accel_max;
    cmd.crc16      = crc16_calc(reinterpret_cast<const uint8_t *>(&cmd), MOVE_CMD_CRC_LEN);

    memcpy(frame, &cmd, FRAME_LEN);
}

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static volatile sig_atomic_t g_run = 1;
static void sigint_handler(int sig) { (void)sig; g_run = 0; }

// Blocks on a line of stdin. Returns false on EOF/Ctrl-D (or a signal
// interrupting the read), which the caller treats as "stop looping".
static bool wait_for_enter(const char *prompt)
{
    printf("\n%s", prompt);
    fflush(stdout);
    char line[64];
    return fgets(line, sizeof(line), stdin) != nullptr;
}

static int capture_one_move(int fd, int gpio_h, uint32_t speed_hz,
                             int32_t target_pos, int32_t vel_max, int32_t accel_max)
{
    // Boot-gate trigger -- same 10ms active-low pulse main.cpp uses. The
    // STM blocks at boot until it sees this, regardless of what firmware
    // mode it's running.
    lgGpioWrite(gpio_h, PIN_FIRE_SYSID, 0);
    usleep(10000);
    lgGpioWrite(gpio_h, PIN_FIRE_SYSID, 1);
    printf("trigger sent, capturing...\n");

    uint8_t move_frame[FRAME_LEN];
    build_move_cmd(move_frame, target_pos, vel_max, accel_max);
    printf("sending MoveCmd (target=%d vel_max=%d accel_max=%d, counts/counts-per-s/counts-per-s^2) "
           "until SERVO_ON confirms it landed...\n", target_pos, vel_max, accel_max);

    struct Sample { double host_time_s; TelemetryFrame frame; };
    std::vector<Sample> samples;
    samples.reserve(20000);

    uint32_t torn_frames    = 0;
    uint32_t move_cmd_sends = 0;
    bool     saw_servo_on   = false;

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

        // Keep resending the MoveCmd every transaction until we see
        // confirmation it was received (drive_state -> SERVO_ON) --
        // sending it exactly once meant a single torn transaction (normal,
        // ~3-5% baseline rate) silently dropped the command with no way to
        // tell and no retry. A corrupted MoveCmd fails its own CRC on the
        // STM and is safely ignored either way, and re-latching the same
        // not-yet-started move is harmless, so resending costs nothing.
        uint8_t rx[FRAME_LEN];
        const uint8_t *tx_override = saw_servo_on ? nullptr : move_frame;
        if (spi_read_frame(fd, speed_hz, rx, tx_override) < 0) break;
        if (tx_override) move_cmd_sends++;

        TelemetryFrame frame;
        memcpy(&frame, rx, FRAME_LEN);

        uint16_t calc = crc16_calc(rx, TELEM_CRC_LEN);
        if (calc != frame.crc16) { torn_frames++; continue; }

        samples.push_back({now_s() - t0, frame});

        if (frame.drive_state == DRIVE_SERVO_ON && !saw_servo_on)
        {
            saw_servo_on = true;
            printf("MoveCmd confirmed received after %u send(s)\n", move_cmd_sends);
        }
        if (saw_servo_on && frame.drive_state == DRIVE_IDLE)
        {
            printf("move complete after %zu frames (%u torn)\n", samples.size(), torn_frames);
            break;
        }
    }

    if (!saw_servo_on)
    {
        fprintf(stderr, "MoveCmd never confirmed received after %u sends (%u torn frames) -- "
                        "check the SPI link, not just this command\n", move_cmd_sends, torn_frames);
    }

    if (samples.empty())
    {
        fprintf(stderr, "no valid frames captured (%u torn) -- was the STM freshly "
                        "flashed with a build that sends TelemetryFrame?\n", torn_frames);
        return -1;
    }

    FILE *csv = fopen(OUT_CSV, "w");
    if (!csv) { perror("fopen csv"); return -1; }

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
    printf("wrote %s (%u torn frames)\n", OUT_CSV, torn_frames);

    int rc = system("python3 " PLOT_SCRIPT);
    (void)rc;

    return 0;
}

int main(int argc, char **argv)
{
    // Usage: move_telem [target_pos] [vel_max] [accel_max] [dev] [speed_hz]
    // First three are counts / counts/s / counts/s^2 (the wire units).
    // target_pos is RELATIVE to wherever the stage is when the move
    // starts -- sending the same command twice moves twice, it does not
    // repeat the same absolute destination. Default matches the values
    // this move was bench-verified with.
    int32_t     target_pos = (argc > 1) ? static_cast<int32_t>(strtol(argv[1], nullptr, 0)) : 3900;
    int32_t     vel_max    = (argc > 2) ? static_cast<int32_t>(strtol(argv[2], nullptr, 0)) : 3900;
    int32_t     accel_max  = (argc > 3) ? static_cast<int32_t>(strtol(argv[3], nullptr, 0)) : 13000;
    const char *dev        = (argc > 4) ? argv[4] : DEFAULT_DEV;
    uint32_t    speed_hz   = (argc > 5) ? static_cast<uint32_t>(strtoul(argv[5], nullptr, 0))
                                         : DEFAULT_SPEED_HZ;

    if (vel_max <= 0 || accel_max <= 0)
    {
        fprintf(stderr, "vel_max and accel_max must be > 0\n");
        return 1;
    }

    static_assert(sizeof(TelemetryFrame) == FRAME_LEN, "TelemetryFrame must be 32 bytes");

    std::signal(SIGINT, sigint_handler);

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

    // Re-arms after every capture instead of exiting -- flash/reset the STM
    // for the next test, then hit Enter here when it's booted and ready.
    // Same idiom as main.cpp's own capture loop.
    while (g_run)
    {
        if (!wait_for_enter("Press Enter once the STM is flashed and ready "
                            "for the next test (Ctrl+C to exit): "))
            break;
        if (!g_run) break;

        capture_one_move(fd, gpio_h, speed_hz, target_pos, vel_max, accel_max);
    }

    close(fd);
    lgGpiochipClose(gpio_h);
    return 0;
}
