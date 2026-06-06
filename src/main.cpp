#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <cstdlib>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <lgpio.h>
#include <fstream>

#include "protocol.h"
#include "profile.hpp"

#define READY_GPIO 7
#define CS_GPIO    25

#define CS_SETUP_US 50
#define CS_GAP_US   200

static uint8_t crc8(const uint8_t* buf, int len)
{
    uint8_t crc = 0;

    for (int i = 0; i < len; i++) {
        crc ^= buf[i];
    }

    return crc;
}

static bool spi_transfer(int gpio_h,
                         int fd,
                         uint8_t* tx,
                         uint8_t* rx,
                         uint32_t speed)
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

    const bool ok = ioctl(fd, SPI_IOC_MESSAGE(1), &tr) >= 0;

    lgGpioWrite(gpio_h, CS_GPIO, 1);
    usleep(CS_GAP_US);

    return ok;
}

static bool send_block_header(int gpio_h,
                              int fd,
                              uint8_t* tx,
                              uint8_t* rx,
                              uint32_t speed,
                              int total_samples)
{
    // flush stale telem from previous run
    for (int i = 0; i < 10; i++)
    {
        std::memset(tx, 0, SPI2_TRANSACTION_BYTES);
        std::memset(rx, 0, SPI2_TRANSACTION_BYTES);
        tx[0] = SPI2_OP_NOP;
        spi_transfer(gpio_h, fd, tx, rx, speed);
        usleep(CS_GAP_US);
    }

    tx[0] = SPI2_OP_BLOCK_HDR;
    tx[1] = static_cast<uint8_t>(total_samples & 0xFF);
    tx[2] = static_cast<uint8_t>((total_samples >> 8) & 0xFF);
    tx[3] = crc8(tx, 3);

    if (!spi_transfer(gpio_h, fd, tx, rx, speed)) {
        perror("spi_transfer BLOCK_HDR");
        return false;
    }

    return true;
}


bool collecting = false;
static bool send_samples(int gpio_h,
                         int fd,
                         uint8_t* tx,
                         uint8_t* rx,
                         uint32_t speed,
                         const std::vector<Sample>& profile,
                         std::vector<TelemetryFrame>& telem)
{
    telem.clear();
    telem.reserve(profile.size());

    for (size_t i = 0; i < profile.size(); i++) {
        const Sample& s = profile[i];

        std::memset(tx, 0, SPI2_TRANSACTION_BYTES);
        std::memset(rx, 0, SPI2_TRANSACTION_BYTES);

        int32_t pos = static_cast<int32_t>(s.pos);
        int32_t vel = static_cast<int32_t>(s.vel);

        tx[0] = SPI2_OP_DATA;
        std::memcpy(&tx[1], &pos, sizeof(int32_t));
        std::memcpy(&tx[5], &vel, sizeof(int32_t));
        tx[9] = crc8(tx, 9);

        if (!spi_transfer(gpio_h, fd, tx, rx, speed))
        {
            std::fprintf(stderr, "spi_transfer DATA failed at sample %zu\n", i);
            return false;
        }

        TelemetryFrame f;
        std::memcpy(&f, rx, sizeof(TelemetryFrame));

        if (f.samples_consumed == 1) collecting = true;

        if (collecting &&
            (telem.empty() || f.samples_consumed > telem.back().samples_consumed))
        {
            telem.push_back(f);
        }

        if (i < 5 || (i % 200) == 0)
        {
            std::printf("TX[%5zu]: pos=%d vel=%d | telem: state=%u ts=%u consumed=%u pos=%d\n",
                i, pos, vel,
                f.drive_state, f.timestamp_ms, f.samples_consumed, f.pos_cmd);
        }
    }

    return true;
}

int main()
{
    const int32_t START_CNT  = 0;
    const int32_t TARGET_CNT = 100000;
    const int32_t VEL_CNT    = 200000;
    const int32_t ACCEL_CNT  = 500000;
    const double  DT         = 0.001;

    std::system("rm -f ../docs/run_*.csv ../docs/run_*.png");

    std::vector<Sample> profile =
        compute_profile(START_CNT, TARGET_CNT, VEL_CNT, ACCEL_CNT, DT);

    const int total_samples = static_cast<int>(profile.size());

    std::printf("profile computed: %d samples\n", total_samples);
    std::printf("  first: pos=%d vel=%d\n",
                profile.front().pos,
                profile.front().vel);
    std::printf("  last:  pos=%d vel=%d\n",
                profile.back().pos,
                profile.back().vel);

    int gpio_h = lgGpiochipOpen(0);
    if (gpio_h < 0) {
        std::fprintf(stderr, "lgGpiochipOpen failed\n");
        return 1;
    }

    int rc = lgGpioClaimInput(gpio_h, LG_SET_PULL_NONE, READY_GPIO);
    if (rc < 0) {
        std::fprintf(stderr, "claim READY failed: %s\n", lguErrorText(rc));
        lgGpiochipClose(gpio_h);
        return 1;
    }

    rc = lgGpioClaimOutput(gpio_h, 0, CS_GPIO, 1);
    if (rc < 0) {
        std::fprintf(stderr, "claim CS failed: %s\n", lguErrorText(rc));
        lgGpiochipClose(gpio_h);
        return 1;
    }

    int fd = open("/dev/spidev0.0", O_RDWR);
    if (fd < 0) {
        perror("open spidev");
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

    std::vector<TelemetryFrame> telem;
    uint32_t run = 0;

    while (true)
    {
        std::printf("press enter to stream profile\n");
        getchar();

        if (!send_block_header(gpio_h, fd, tx, rx, speed, total_samples)) {
            continue;
        }

        usleep(200);

        if (!send_samples(gpio_h, fd, tx, rx, speed, profile, telem))
        {
            continue;
        }

        while (!telem.empty() && telem.back().samples_consumed < (uint32_t)total_samples)
        {
            std::memset(tx, 0, SPI2_TRANSACTION_BYTES);
            std::memset(rx, 0, SPI2_TRANSACTION_BYTES);
        
            int32_t pos = TARGET_CNT;
            int32_t vel = 0;
        
            tx[0] = SPI2_OP_DATA;
            std::memcpy(&tx[1], &pos, sizeof(int32_t));
            std::memcpy(&tx[5], &vel, sizeof(int32_t));
            tx[9] = crc8(tx, 9);
        
            spi_transfer(gpio_h, fd, tx, rx, speed);
        
            TelemetryFrame f;
            std::memcpy(&f, rx, sizeof(TelemetryFrame));
            if (f.samples_consumed > 0 &&
                (telem.empty() || f.timestamp_ms != telem.back().timestamp_ms))
            {
                telem.push_back(f);
            }
        }


        std::printf("done. sent=%d\n", total_samples);
        char fname[64];
        std::snprintf(fname, sizeof(fname), "../docs/run_%03u.csv", run++);
        std::ofstream csv(fname);
        csv << "t,pos_cmd,pos_fbk,vel_cmd,vel_fbk,pos_err,iq_cmd,i_q_fbk,consumed\n";
        uint32_t t0 = telem.empty() ? 0 : telem.front().timestamp_ms;
        for (const auto& f : telem)
        {
            int32_t pos_err = f.pos_cmd - f.pos_fbk;
            int32_t vel_err = f.vel_cmd - f.vel_fbk;
            csv << (f.timestamp_ms - t0) << ","
                << f.pos_cmd             << ","
                << f.pos_fbk             << ","
                << f.vel_cmd             << ","
                << f.vel_fbk             << ","
                << pos_err               << ","
                << f.iq_cmd              << ","
                << f.i_q_fbk             << ","
                << f.samples_consumed    << "\n";
        }
        std::printf("wrote %s\n", fname);

        char cmd[128];
        std::snprintf(cmd, sizeof(cmd), "python3 ../py-script/plotprof.py %s", fname);
        std::system(cmd);
    }

    lgGpioWrite(gpio_h, CS_GPIO, 1);
    close(fd);
    lgGpiochipClose(gpio_h);
    return 0;
}