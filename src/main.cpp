#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <lgpio.h>
#include "protocol.h"

#define READY_GPIO 25

static uint8_t crc8(const uint8_t* buf, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) crc ^= buf[i];
    return crc;
}

static bool spi_transfer(int fd, uint8_t* tx, uint8_t* rx, uint32_t speed)
{
    spi_ioc_transfer tr = {};
    tr.tx_buf        = reinterpret_cast<unsigned long>(tx);
    tr.rx_buf        = reinterpret_cast<unsigned long>(rx);
    tr.len           = 32;
    tr.speed_hz      = speed;
    tr.bits_per_word = 8;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &tr) >= 0;
}

static void process_telem(const uint8_t* rx, bool verbose)
{
    static uint32_t last_print_ms = 0;
    static uint8_t  last_state    = 0xFF;
    static bool     raw_done      = false;

    TelemetryFrame tf;
    memcpy(&tf, rx, sizeof(tf));

    if (verbose && !raw_done) {
        printf("RAW[0..31]: ");
        for (int i = 0; i < 32; i++) printf("%02X ", rx[i]);
        printf("  consumed=%u\n", tf.samples_consumed);
        if (tf.samples_consumed >= 2048u)
            raw_done = true;
    }

    bool state_changed = (tf.drive_state != last_state);
    bool time_elapsed  = (tf.timestamp_ms - last_print_ms) >= 100;

    if (state_changed || time_elapsed) {
        printf("[t=%6u ms]  state=%u  consumed=%6u  pos=%d  vel=%d  faults=0x%02X\n",
               tf.timestamp_ms, tf.drive_state, tf.samples_consumed,
               tf.pos_cmd, tf.vel_cmd, tf.fault_flags);
        last_print_ms = tf.timestamp_ms;
        last_state    = tf.drive_state;
    }
}

static void send_data_block(int fd, uint8_t* tx, uint8_t* rx, uint32_t speed,
                            int start, int count)
{
    for (int i = 0; i < count; i++) {
        memset(tx, 0, 32);
        tx[0] = SPI2_OP_DATA;
        int32_t pos = (start + i) * 10;
        int32_t vel = 10;
        memcpy(&tx[1], &pos, 4);
        memcpy(&tx[5], &vel, 4);
        tx[9] = crc8(tx, 9);
        spi_transfer(fd, tx, rx, speed);
        process_telem(rx, false);
    }
}

int main()
{
    int gpio_h = lgGpiochipOpen(0);
    if (gpio_h < 0) { fprintf(stderr, "lgGpiochipOpen failed\n"); return 1; }
    lgGpioClaimInput(gpio_h, LG_SET_PULL_NONE, READY_GPIO);

    int fd = open("/dev/spidev0.0", O_RDWR);
    if (fd < 0) { perror("open"); return 1; }

    uint8_t  mode  = SPI_MODE_0;
    uint8_t  bits  = 8;
    uint32_t speed = 1000000;
    ioctl(fd, SPI_IOC_WR_MODE,          &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ,  &speed);

    uint8_t tx[32] = {};
    uint8_t rx[32] = {};

    const int TOTAL_SAMPLES = 2048;
    const int INITIAL_FILL  = (TOTAL_SAMPLES < 4096) ? TOTAL_SAMPLES : 4096;
    const int REFILL_SIZE   = 2048;

    while (true)
    {
        // ── Wait for STM ready ────────────────────────────────────────────────
        printf("waiting for STM ready...\n");
        while (lgGpioRead(gpio_h, READY_GPIO) == 0);
        printf("STM ready — press enter to start profile\n");
        getchar();

        // ── BLOCK_HDR ─────────────────────────────────────────────────────────
        memset(tx, 0, 32);
        tx[0] = SPI2_OP_BLOCK_HDR;
        tx[1] = (TOTAL_SAMPLES >> 8) & 0xFF;
        tx[2] =  TOTAL_SAMPLES       & 0xFF;
        tx[3] = crc8(tx, 3);
        spi_transfer(fd, tx, rx, speed);
        process_telem(rx, false);

        // ── Initial fill ──────────────────────────────────────────────────────
        send_data_block(fd, tx, rx, speed, 0, INITIAL_FILL);
        printf("initial fill sent: %d samples\n", INITIAL_FILL);

        int total_sent = INITIAL_FILL;

        // ── Refill loop — watch READY pin ─────────────────────────────────────
        while (total_sent < TOTAL_SAMPLES)
        {
            while (lgGpioRead(gpio_h, READY_GPIO) != 0);
            int remaining = TOTAL_SAMPLES - total_sent;
            int to_send   = (remaining < REFILL_SIZE) ? remaining : REFILL_SIZE;
            send_data_block(fd, tx, rx, speed, total_sent, to_send);
            total_sent += to_send;
            printf("refill: sent %d, total_sent=%d\n", to_send, total_sent);
        }

        // ── Poll until move complete — drive returns to IDLE ──────────────────
        printf("all samples sent, polling for move complete...\n");
        TelemetryFrame tf{};
        while (true)
        {
            memset(tx, 0, 32);
            tx[0] = SPI2_OP_TELEM_REQ;
            tx[1] = crc8(tx, 1);
            spi_transfer(fd, tx, rx, speed);
            memcpy(&tf, rx, sizeof(tf));
            process_telem(rx, true);

            if (tf.samples_consumed >= static_cast<uint32_t>(TOTAL_SAMPLES))// &&
                //tf.drive_state == DRIVE_IDLE)
                break;

            usleep(10000);   // 100 Hz poll
        }

        printf("\n=== move complete ===\n");
        printf("  samples_consumed = %u  (expected %d)\n",
               tf.samples_consumed, TOTAL_SAMPLES);
        printf("  drive_state      = %u\n", tf.drive_state);
        printf("  fault_flags      = 0x%02X\n", tf.fault_flags);
        printf("  result: %s\n",
               (tf.samples_consumed == static_cast<uint32_t>(TOTAL_SAMPLES) &&
                tf.fault_flags == 0) ? "PASS" : "FAIL");
    }

    lgGpiochipClose(gpio_h);
    close(fd);
    return 0;
}