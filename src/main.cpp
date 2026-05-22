#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

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

int main()
{
    int fd = open("/dev/spidev0.0", O_RDWR);
    if (fd < 0) { perror("open"); return 1; }

    uint8_t mode  = SPI_MODE_0;
    uint8_t bits  = 8;
    uint32_t speed = 1000000;

    ioctl(fd, SPI_IOC_WR_MODE,          &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ,  &speed);

    uint8_t tx[32] = {};
    uint8_t rx[32] = {};

    int block = 0;

    while (true)
    {
        // ── BLOCK_HDR ─────────────────────────────────────────────────────
        memset(tx, 0, 32);
        tx[0] = 0x03;           // SPI2_OP_BLOCK_HDR
        tx[1] = 0x00;           // sample count high
        tx[2] = 128;            // sample count low = 128
        tx[3] = crc8(tx, 3);
        spi_transfer(fd, tx, rx, speed);

        // ── 128 DATA packets ──────────────────────────────────────────────
        for (int i = 0; i < 128; i++)
        {
            memset(tx, 0, 32);
            tx[0] = 0x04;                       // SPI2_OP_DATA
            int32_t pos = i * 10;               // simple ramp
            int32_t vel = 10;
            memcpy(&tx[1], &pos, 4);
            memcpy(&tx[5], &vel, 4);
            tx[9] = crc8(tx, 9);
            spi_transfer(fd, tx, rx, speed);
        }

        printf("block %d sent\n", ++block);
    }

    close(fd);
    return 0;
}