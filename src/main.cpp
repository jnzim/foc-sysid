#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <lgpio.h>
#include "protocol.h"
#include "profile.hpp"

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

static void dump_rx(const uint8_t* rx)
{
    static int count = 0;
    if (count < 5 || (count % 200) == 0) {
        printf("RX[%5d]: ", count);
        for (int i = 0; i < 10; i++) printf("%02X ", rx[i]);
        printf("\n");
    }
    count++;
}

// ── Loopback diff ────────────────────────────────────────────────────────
// STM echoes prior frame, so rx[N] should equal tx[N-1] in bytes 0..9.
struct LoopbackChecker {
    uint8_t prev_tx[10] = {};
    bool    armed       = false;
    int     ok_count    = 0;
    int     mismatch    = 0;
};

static void check_loopback(LoopbackChecker& lc, const uint8_t* tx, const uint8_t* rx)
{
    if (lc.armed) {
        if (memcmp(lc.prev_tx, rx, 10) != 0) {
            if (lc.mismatch < 10) {
                printf("MISMATCH #%d\n  expected: ", lc.mismatch);
                for (int i = 0; i < 10; i++) printf("%02X ", lc.prev_tx[i]);
                printf("\n  got:      ");
                for (int i = 0; i < 10; i++) printf("%02X ", rx[i]);
                printf("\n");
            }
            lc.mismatch++;
        } else {
            lc.ok_count++;
        }
    }
    memcpy(lc.prev_tx, tx, 10);
    lc.armed = true;
}

static void send_samples(int gpio_h, int fd, uint8_t* tx, uint8_t* rx, uint32_t speed,
                         const std::vector<Sample>& profile, int start, int count,
                         LoopbackChecker& lc)
{
    for (int i = 0; i < count; i++) {
        const Sample& s = profile[start + i];

        memset(tx, 0, SPI2_TRANSACTION_BYTES);
        memset(rx, 0, SPI2_TRANSACTION_BYTES);

        tx[0] = SPI2_OP_DATA;
        int32_t pos = s.pos;
        int32_t vel = s.vel;
        memcpy(&tx[1], &pos, sizeof(int32_t));
        memcpy(&tx[5], &vel, sizeof(int32_t));
        tx[9] = crc8(tx, 9);

        if (!spi_transfer(gpio_h, fd, tx, rx, speed)) {
            perror("spi_transfer DATA");
            return;
        }

        check_loopback(lc, tx, rx);
        dump_rx(rx);
    }
}

int main()
{
    // ── Compute profile FIRST ────────────────────────────────────────────
    // Tuned to produce ~2000 samples at dt = 1ms: triangular profile,
    // tTotal = 2*sqrt(dist/accel) = 2s → 2001 samples.
    const int32_t START_CNT  = 0;
    const int32_t TARGET_CNT = 100000;
    const int32_t VEL_CNT    = 200000;   // high enough to force triangular
    const int32_t ACCEL_CNT  = 100000;
    const double  DT         = 0.001;

    std::vector<Sample> profile =
        compute_profile(START_CNT, TARGET_CNT, VEL_CNT, ACCEL_CNT, DT);

    const int TOTAL_SAMPLES = static_cast<int>(profile.size());

    printf("profile computed: %d samples\n", TOTAL_SAMPLES);
    printf("  first: pos=%d vel=%d\n", profile.front().pos, profile.front().vel);
    printf("  last:  pos=%d vel=%d\n", profile.back().pos,  profile.back().vel);

    // ── GPIO + SPI setup ─────────────────────────────────────────────────
    int gpio_h = lgGpiochipOpen(0);
    if (gpio_h < 0) { fprintf(stderr, "lgGpiochipOpen failed\n"); return 1; }

    int rc = lgGpioClaimInput(gpio_h, LG_SET_PULL_NONE, READY_GPIO);
    if (rc < 0) {
        fprintf(stderr, "claim READY failed: %s\n", lguErrorText(rc));
        lgGpiochipClose(gpio_h);
        return 1;
    }

    rc = lgGpioClaimOutput(gpio_h, 0, CS_GPIO, 1);
    if (rc < 0) {
        fprintf(stderr, "claim CS failed: %s\n", lguErrorText(rc));
        lgGpiochipClose(gpio_h);
        return 1;
    }

    int fd = open("/dev/spidev0.0", O_RDWR);
    if (fd < 0) { perror("open spidev"); lgGpiochipClose(gpio_h); return 1; }

    uint8_t  mode  = SPI_MODE_1;
    uint8_t  bits  = 8;
    uint32_t speed = 1000000;
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    uint8_t tx[SPI2_TRANSACTION_BYTES] = {};
    uint8_t rx[SPI2_TRANSACTION_BYTES] = {};

    while (true)
    {
        printf("press enter to stream profile\n");
        getchar();

        LoopbackChecker lc;

        // ── BLOCK_HDR ────────────────────────────────────────────────────
        memset(tx, 0, SPI2_TRANSACTION_BYTES);
        memset(rx, 0, SPI2_TRANSACTION_BYTES);
        tx[0] = SPI2_OP_BLOCK_HDR;
        tx[1] = static_cast<uint8_t>(TOTAL_SAMPLES & 0xFF);
        tx[2] = static_cast<uint8_t>((TOTAL_SAMPLES >> 8) & 0xFF);
        tx[3] = crc8(tx, 3);

        if (!spi_transfer(gpio_h, fd, tx, rx, speed)) {
            perror("BLOCK_HDR");
            continue;
        }
        check_loopback(lc, tx, rx);
        dump_rx(rx);
        usleep(200);

        // ── Stream all 2000 samples in one shot (fits in 4096-deep ring) ─
        send_samples(gpio_h, fd, tx, rx, speed, profile, 0, TOTAL_SAMPLES, lc);

        printf("done. sent=%d  loopback ok=%d  mismatch=%d\n",
               TOTAL_SAMPLES, lc.ok_count, lc.mismatch);
    }

    lgGpioWrite(gpio_h, CS_GPIO, 1);
    close(fd);
    lgGpiochipClose(gpio_h);
    return 0;
}