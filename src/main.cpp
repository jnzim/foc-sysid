

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
#include "config.hpp"

#define READY_REFILL_GPIO 7
#define CS_GPIO    25

#define CS_SETUP_US 50
#define CS_GAP_US   200


// Profile data
float start_mm              = 0;
float target_mm             = 125;
float vel_mm_sec            = 20;
float acel_mm_sec_sec       = 250;
const double  DT            = 0.001;
int START_CNTS              = 0;
int TARGET_CNTS             = 0;
int VEL_CNTS_PER_SEC        = 0;
int ACEL_CNTS_PER_SEC_SEC   = 0;


static bool spi_transfer(int gpio_h,
                         int fd,
                         uint8_t* tx,
                         uint8_t* rx,
                         uint32_t speed)
{
    // lgGpioWrite(gpio_h, CS_GPIO, 0);
    // usleep(CS_SETUP_US);

    spi_ioc_transfer tr = {};
    tr.tx_buf        = reinterpret_cast<unsigned long>(tx);
    tr.rx_buf        = reinterpret_cast<unsigned long>(rx);
    tr.len           = SPI2_TRANSACTION_BYTES;
    tr.speed_hz      = speed;
    tr.bits_per_word = 8;
    tr.cs_change     = 0;

    const bool ok = ioctl(fd, SPI_IOC_MESSAGE(1), &tr) >= 0;

    // lgGpioWrite(gpio_h, CS_GPIO, 1);
    // usleep(CS_GAP_US);

    return ok;
}

static bool send_block_header(int gpio_h,
                              int fd,
                              uint8_t* tx,
                              uint8_t* rx,
                              uint32_t speed,
                              int total_samples)
{
    /* Flush stale telem */
    for (int i = 0; i < 10; i++)
    {
        std::memset(tx, 0, SPI2_TRANSACTION_BYTES);
        std::memset(rx, 0, SPI2_TRANSACTION_BYTES);
        tx[0] = SPI2_OP_NOP;
        spi_transfer(gpio_h, fd, tx, rx, speed);
        usleep(CS_GAP_US);
    }

    /* Build and send BLOCK_HDR packet */
    std::memset(tx, 0, SPI2_TRANSACTION_BYTES);
    std::memset(rx, 0, SPI2_TRANSACTION_BYTES);

    tx[0] = SPI2_OP_BLOCK_HDR;
    tx[1] = static_cast<uint8_t>(total_samples & 0xFF);
    tx[2] = static_cast<uint8_t>((total_samples >> 8) & 0xFF);
    
    /* Calculate CRC-16 over bytes 0..13 */
    uint16_t crc = crc16_calc(tx, 14);
    tx[14] = static_cast<uint8_t>(crc & 0xFF);
    tx[15] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    std::printf("BLOCK_HDR: samples=%d crc16=%04x\n", total_samples, crc);

    if (!spi_transfer(gpio_h, fd, tx, rx, speed)) {
        perror("spi_transfer BLOCK_HDR");
        return false;
    }

    return true;
}

bool collecting = false;
static bool send_samples(int gpio_h, int fd,
                         uint8_t* tx,
                         uint8_t* rx,
                         uint32_t speed,
                         const std::vector<Sample>& profile,
                         size_t profile_offset,     
                         size_t block_size,         
                         std::vector<TelemetryFrame>& telem)
{
    // Don't clear telem — append to existing telemetry
    telem.reserve(telem.size() + block_size);
    
    for (size_t i = 0; i < block_size; i++) 
    {
        const Sample& s = profile[profile_offset + i]; // keep track of where we are
        
        std::memset(tx, 0, SPI2_TRANSACTION_BYTES);
        std::memset(rx, 0, SPI2_TRANSACTION_BYTES);
        
        TrajSlot slot;
        slot.opcode  = SPI2_OP_DATA;
        slot.seq     = static_cast<uint8_t>(i & 0xFF);
        slot.pos_cmd = static_cast<int32_t>(s.pos);
        slot.vel_cmd = static_cast<int32_t>(s.vel);
        slot.reserved = 0;
        slot.crc16 = crc16_calc((uint8_t*)&slot, TRAJ_CRC_LEN);
        
        std::memcpy(tx, &slot, sizeof(TrajSlot));
        if (!spi_transfer(gpio_h, fd, tx, rx, speed)) {
            std::fprintf(stderr, "spi_transfer DATA failed at sample %zu\n", i);
            return false;
        }
        
        TelemetryFrame f;
        std::memcpy(&f, rx, sizeof(TelemetryFrame));
        if (f.samples_consumed == 1) collecting = true;
        
        if (collecting && (telem.empty() || f.samples_consumed > telem.back().samples_consumed)) {
            telem.push_back(f);
        }
        
        // if (i < 5 || (i % 200) == 0) {
        //     std::printf("TX[%5zu]: pos=%d vel=%d | telem: state=%u consumed=%u\n",
        //                 i, slot.pos_cmd, slot.vel_cmd, f.drive_state, f.samples_consumed);
        //}
    }
    return true;
}
static bool drain(int gpio_h, int fd, uint8_t* tx, uint8_t* rx,  uint32_t speed, std::vector<TelemetryFrame>& telem)
{
    static uint8_t drain_seq = 0;
    
    std::memset(tx, 0, SPI2_TRANSACTION_BYTES);
    std::memset(rx, 0, SPI2_TRANSACTION_BYTES);
    
    /* Build TrajSlot with telemetry request */
    TrajSlot slot;
    slot.opcode     = SPI2_OP_TELEM_REQ;
    slot.seq        = drain_seq++;
    slot.pos_cmd    = TARGET_CNTS;
    slot.vel_cmd    = 0;
    slot.reserved   = 0;
    slot.crc16 = crc16_calc((uint8_t*)&slot, TRAJ_CRC_LEN);
    
    std::memcpy(tx, &slot, sizeof(TrajSlot));
    
    if (!spi_transfer(gpio_h, fd, tx, rx, speed)) 
    {
        std::fprintf(stderr, "spi_transfer TELEM_REQ failed\n");
        return false;
    }
    
    TelemetryFrame f;
    std::memcpy(&f, rx, sizeof(TelemetryFrame));
    
    if (f.samples_consumed > 0 && (telem.empty() || f.timestamp_ms != telem.back().timestamp_ms)) 
    {
        telem.push_back(f);
    }
    
    return true;
}


int main()
{


    std::system("rm -f ../docs/run_*.csv ../docs/run_*.png");
    START_CNTS              = MACHINE.mm_to_counts(start_mm);
    TARGET_CNTS             = MACHINE.mm_to_counts(target_mm);
    VEL_CNTS_PER_SEC        =  MACHINE.mm_to_counts(vel_mm_sec);
    ACEL_CNTS_PER_SEC_SEC   = MACHINE.mm_to_counts(acel_mm_sec_sec);
    std::vector<Sample> profile = compute_profile(START_CNTS, TARGET_CNTS, VEL_CNTS_PER_SEC, ACEL_CNTS_PER_SEC_SEC, DT);

    const int total_samples = static_cast<int>(profile.size());


    {
        std::ofstream profile_csv("../docs/profile_sent.csv");
        profile_csv << "t,pos,vel\n";
        for (size_t i = 0; i < profile.size(); i++) {
            profile_csv << (i * DT) << "," 
                        << profile[i].pos << "," 
                        << profile[i].vel << "\n";
        }
        profile_csv.close();
    }

    
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

    int rc = lgGpioClaimInput(gpio_h, LG_SET_PULL_NONE, READY_REFILL_GPIO);
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
    size_t profile_offset   = 0;
    uint32_t run            = 0;

    while (true)
    {
        std::printf("press enter to stream profile\n");
        getchar();

        if (!send_block_header(gpio_h, fd, tx, rx, speed, total_samples)) 
        {
            continue;
        }

        usleep(200);
        int block_size = 0;

        if (profile.size() >= 2048)         { block_size = 2048; }
        else                                { block_size = profile.size();}
        
        if (!send_samples(gpio_h, fd, tx, rx, speed, profile, profile_offset, block_size,  telem))
        {
            continue;
        }
        profile_offset = block_size;

        //Drain refill loop: 
        // keep sending final position until STM finishes 
        // On ready READY_REFILL_GPIO, send new 1/2 block
        {
            while (profile_offset < profile.size() || telem.back().samples_consumed < profile.size()) 
            {
                int ready = lgGpioRead(gpio_h, READY_REFILL_GPIO);

                if (ready == 1 && profile_offset < profile.size()) 
                {
                    // Send next 1024
                    size_t num_profile_frames = std::min(size_t(1024), profile.size() - profile_offset);
                    send_samples(gpio_h, fd, tx, rx, speed, profile, profile_offset, num_profile_frames, telem);
                    profile_offset += num_profile_frames;
                } 
                else
                {
                    // Drain telemetry
                    drain(gpio_h, fd, tx, rx, speed, telem);
                }
            }

        }

        std::printf("done. sent=%d\n", total_samples);
        char fname[64];
        std::snprintf(fname, sizeof(fname), "../docs/run_%03u.csv", run++);
        std::ofstream csv(fname);
        csv << "t,pos_cmd,pos_fbk,vel_cmd,vel_fbk,pos_err,vel_err,iq_cmd,i_q_fbk,v_q_cmd,consumed\n";

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
                << vel_err               << ","
                << f.iq_cmd              << ","
                << f.i_q_fbk             << ","
                << f.v_q_cmd             << ","
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