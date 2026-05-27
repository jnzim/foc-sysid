#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <lgpio.h>
#include "protocol.h"

#define READY_GPIO 7
#define CS_GPIO    25

#define CS_SETUP_US 50
#define CS_GAP_US   10

static uint8_t crc8(const uint8_t* buf, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= buf[i];
    }
    return crc;
}

static bool spi_transfer(int gpio_h, int fd, uint8_t* tx, uint8_t* rx, uint32_t speed)
{
    lgGpioWrite(gpio_h, CS_GPIO, 0);
    usleep(CS_SETUP_US);

    spi_ioc_transfer tr = {};
    tr.tx_buf        = reinterpret_cast<unsigned long>(tx);
    tr.rx_buf        = reinterpret_cast<unsigned long>(rx);
    tr.len           = SPI2_TRANSACTION_BYTES;
    tr.speed_hz      = speed;
    tr.bits_per_word = 8;
    tr.cs_change     = 0;

    bool ok = ioctl(fd, SPI_IOC_MESSAGE(1), &tr) >= 0;

    lgGpioWrite(gpio_h, CS_GPIO, 1);
    usleep(CS_GAP_US);

    return ok;
}

// ── DEBUG: dump raw 32 bytes returned on MISO ────────────────────────────
// Print first 5 frames, then every 100th to keep the terminal usable.
static void process_telem(const uint8_t* rx, bool verbose)
{
    (void)verbose;
    static int count = 0;

    if (count < 5 || (count % 100) == 0) {
        printf("RX[%5d]: ", count);
        for (int i = 0; i < SPI2_TRANSACTION_BYTES; i++) {
            printf("%02X ", rx[i]);
        }
        printf("\n");
    }
    count++;
}

static void send_data_block(int gpio_h, int fd, uint8_t* tx, uint8_t* rx,
                            uint32_t speed, int start, int count)
{
    for (int i = 0; i < count; i++) {
        memset(tx, 0, SPI2_TRANSACTION_BYTES);
        memset(rx, 0, SPI2_TRANSACTION_BYTES);

        /*
         * DATA packet, 32-bit version:
         * [0]   opcode
         * [1-4] int32 pos_cmd
         * [5-8] int32 vel_cmd
         * [9]   CRC over bytes 0-8
         */
        int32_t pos = static_cast<int32_t>(start + i);
        int32_t vel = 10;

        tx[0] = SPI2_OP_DATA;
        memcpy(&tx[1], &pos, sizeof(int32_t));
        memcpy(&tx[5], &vel, sizeof(int32_t));
        tx[9] = crc8(tx, 9);

        if (!spi_transfer(gpio_h, fd, tx, rx, speed)) {
            perror("spi_transfer DATA");
            return;
        }

        process_telem(rx, false);
    }
}

int main()
{
    int gpio_h = lgGpiochipOpen(0);
    if (gpio_h < 0) {
        fprintf(stderr, "lgGpiochipOpen failed\n");
        return 1;
    }

    int rc = lgGpioClaimInput(gpio_h, LG_SET_PULL_NONE, READY_GPIO);
    if (rc < 0) {
        fprintf(stderr, "lgGpioClaimInput READY failed: %s\n", lguErrorText(rc));
        lgGpiochipClose(gpio_h);
        return 1;
    }

    rc = lgGpioClaimOutput(gpio_h, 0, CS_GPIO, 1);   // CS idle high
    if (rc < 0) {
        fprintf(stderr, "lgGpioClaimOutput CS failed: %s\n", lguErrorText(rc));
        lgGpiochipClose(gpio_h);
        return 1;
    }

    int fd = open("/dev/spidev0.0", O_RDWR);
    if (fd < 0) {
        perror("open /dev/spidev0.0");
        lgGpiochipClose(gpio_h);
        return 1;
    }

    uint8_t  mode  = SPI_MODE_1;
    uint8_t  bits  = 8;
    uint32_t speed = 1000000;

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("SPI_IOC_WR_MODE");
        close(fd);
        lgGpiochipClose(gpio_h);
        return 1;
    }

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("SPI_IOC_WR_BITS_PER_WORD");
        close(fd);
        lgGpiochipClose(gpio_h);
        return 1;
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("SPI_IOC_WR_MAX_SPEED_HZ");
        close(fd);
        lgGpiochipClose(gpio_h);
        return 1;
    }

    uint8_t tx[SPI2_TRANSACTION_BYTES] = {};
    uint8_t rx[SPI2_TRANSACTION_BYTES] = {};

    const int TOTAL_SAMPLES = 2048;
    const int INITIAL_FILL  = (TOTAL_SAMPLES < 4096) ? TOTAL_SAMPLES : 4096;
    const int REFILL_SIZE   = 2048;

    while (true)
    {
        printf("STM ready / idle check — press enter to start profile\n");
        getchar();

        // ── BLOCK_HDR ─────────────────────────────────────────────────────
        memset(tx, 0, SPI2_TRANSACTION_BYTES);
        memset(rx, 0, SPI2_TRANSACTION_BYTES);

        tx[0] = SPI2_OP_BLOCK_HDR;
        tx[1] = 0;
        tx[2] = 0;
        tx[3] = crc8(tx, 3);

        if (!spi_transfer(gpio_h, fd, tx, rx, speed)) {
            perror("spi_transfer BLOCK_HDR");
            continue;
        }

        process_telem(rx, false);

        usleep(200);

        // ── Initial fill ──────────────────────────────────────────────────
        send_data_block(gpio_h, fd, tx, rx, speed, 0, INITIAL_FILL);
        printf("initial fill sent: %d samples\n", INITIAL_FILL);

        int total_sent = INITIAL_FILL;

        // ── Refill loop ───────────────────────────────────────────────────
        while (total_sent < TOTAL_SAMPLES)
        {
            while (lgGpioRead(gpio_h, READY_GPIO) != 0) {
                usleep(100);
            }

            int remaining = TOTAL_SAMPLES - total_sent;
            int to_send   = (remaining < REFILL_SIZE) ? remaining : REFILL_SIZE;

            send_data_block(gpio_h, fd, tx, rx, speed, total_sent, to_send);

            total_sent += to_send;

            printf("refill: sent %d, total_sent=%d\n", to_send, total_sent);

            while (lgGpioRead(gpio_h, READY_GPIO) == 0) {
                usleep(50);
            }
        }

        printf("all samples sent\n");
    }

    lgGpioWrite(gpio_h, CS_GPIO, 1);
    close(fd);
    lgGpiochipClose(gpio_h);

    return 0;
}