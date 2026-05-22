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

    const int TOTAL_SAMPLES = 8192;
    const int INITIAL_FILL  = 4096;
    const int REFILL_SIZE   = 2048;

    // Wait for STM ready
    printf("waiting for STM ready...\n");
    while (lgGpioRead(gpio_h, READY_GPIO) == 0);
    printf("STM ready — press enter to start profile\n");
    getchar();

    // BLOCK_HDR
    memset(tx, 0, 32);
    tx[0] = SPI2_OP_BLOCK_HDR;
    tx[1] = (TOTAL_SAMPLES >> 8) & 0xFF;
    tx[2] =  TOTAL_SAMPLES       & 0xFF;
    tx[3] = crc8(tx, 3);
    spi_transfer(fd, tx, rx, speed);

    // Initial fill — send all 4096, don't watch READY yet
    send_data_block(fd, tx, rx, speed, 0, INITIAL_FILL);
    printf("initial fill sent: %d samples\n", INITIAL_FILL);

    int total_sent = INITIAL_FILL;

    // Refill loop — watch READY pin
    while (total_sent < TOTAL_SAMPLES)
    {
        while (lgGpioRead(gpio_h, READY_GPIO) != 0);  // wait for READY low
        int remaining = TOTAL_SAMPLES - total_sent;
        int to_send   = (remaining < REFILL_SIZE) ? remaining : REFILL_SIZE;
        send_data_block(fd, tx, rx, speed, total_sent, to_send);
        total_sent += to_send;
        printf("refill: sent %d, total_sent=%d\n", to_send, total_sent);
    }

    printf("all samples sent, waiting for move complete...\n");
    while (lgGpioRead(gpio_h, READY_GPIO) != 0);
    printf("move complete\n");

    lgGpiochipClose(gpio_h);
    close(fd);
    return 0;
}