#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

static void print_bytes(const char* label, const uint8_t* data, int len)
{
    std::printf("%s", label);
    for (int i = 0; i < len; i++)
        std::printf("0x%02X ", data[i]);
    std::printf("\n");
}

int main()
{
    int fd = open("/dev/spidev0.0", O_RDWR);
    if (fd < 0) { perror("open /dev/spidev0.0"); return 1; }

    uint8_t  mode  = SPI_MODE_0;
    uint8_t  bits  = 8;
    uint32_t speed = 1000000;

    if (ioctl(fd, SPI_IOC_WR_MODE,          &mode)  < 0) { perror("mode");  close(fd); return 1; }
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)  < 0) { perror("bits");  close(fd); return 1; }
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ,  &speed) < 0) { perror("speed"); close(fd); return 1; }

    uint8_t tx[32] = {};
    tx[0] = 0x04;   // SPI2_OP_DATA
    tx[1] = 0x01;   // pos_cmd = 1 (little-endian)
    tx[2] = 0x00;
    tx[3] = 0x00;
    tx[4] = 0x00;
    tx[5] = 0x00;   // vel_cmd = 0
    tx[6] = 0x00;
    tx[7] = 0x00;
    tx[8] = 0x00;
    uint8_t crc = 0;
    for (int i = 0; i < 9; i++) crc ^= tx[i];
    tx[9] = crc;

    uint8_t rx[32] = {};

    spi_ioc_transfer tr = {};
    tr.tx_buf        = reinterpret_cast<unsigned long>(tx);
    tr.rx_buf        = reinterpret_cast<unsigned long>(rx);
    tr.len           = 32;
    tr.speed_hz      = speed;
    tr.bits_per_word = bits;
    tr.delay_usecs   = 0;
    tr.cs_change     = 0;

    std::printf("SPI device: /dev/spidev0.0\n");
    std::printf("Speed: %u Hz\n\n", speed);

    while (true)
    {
        std::memset(rx, 0, sizeof(rx));
        int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        printf("ioctl ret: %d\n", ret);
        if (ret < 0) { perror("SPI_IOC_MESSAGE"); break; }
        print_bytes("sent: ", tx, 32);
        print_bytes("got:  ", rx, 32);
        std::printf("press enter to send again\n");
        getchar();
    }

    close(fd);
    return 0;
}