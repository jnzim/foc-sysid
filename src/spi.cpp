// spi.cpp — Pi-side SPI2 protocol implementation
// Raspberry Pi 5, spidev + lgpio, C++17

#include "spi.hpp"
#include "protocol.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/spi/spidev.h>
#include <lgpio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

static constexpr size_t TRANSACTION_BYTES = SPI2_TRANSACTION_BYTES;

static constexpr int READY_GPIO_PIN = 7;    // STM READY, active-low
static constexpr int CS_GPIO_PIN    = 25;   // manual CS / STM NSS

static constexpr size_t INITIAL_FILL_FRAMES = 4096;
static constexpr size_t REFILL_FRAMES       = 2048;

static int spi_fd = -1;
static int gpio_h = -1;

static uint32_t spi_speed_hz_cached = 1000000;

static uint8_t crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0x00;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
    }

    return crc;
}

// =============================================================================
// Raw 32-byte SPI transfer — manual CS via GPIO
// =============================================================================
bool spi_transfer_raw(const uint8_t* tx, uint8_t* rx, size_t len)
{
    if (spi_fd < 0 || gpio_h < 0) {
        return false;
    }

    if (tx == nullptr || rx == nullptr) {
        std::cerr << "spi_transfer_raw: null buffer\n";
        return false;
    }

    if (len != TRANSACTION_BYTES) {
        std::cerr << "spi_transfer_raw: invalid length " << len << "\n";
        return false;
    }

    lgGpioWrite(gpio_h, CS_GPIO_PIN, 0);
    usleep(CS_SETUP_US);

    struct spi_ioc_transfer tr = {};
    tr.tx_buf        = reinterpret_cast<unsigned long>(tx);
    tr.rx_buf        = reinterpret_cast<unsigned long>(rx);
    tr.len           = static_cast<__u32>(len);
    tr.speed_hz      = spi_speed_hz_cached;
    tr.bits_per_word = 8;
    tr.cs_change     = 0;
    tr.delay_usecs   = 0;

    const bool ok = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) >= 0;

    lgGpioWrite(gpio_h, CS_GPIO_PIN, 1);
    usleep(CS_GAP_US);

    return ok;
}

// =============================================================================
// spi_init
// =============================================================================
bool spi_init(const char* device, uint32_t speed_hz)
{
    spi_speed_hz_cached = speed_hz;

    gpio_h = lgGpiochipOpen(0);
    if (gpio_h < 0) {
        std::cerr << "spi_init: lgGpiochipOpen failed: "
                  << lguErrorText(gpio_h) << "\n";
        return false;
    }

    int rc = lgGpioClaimInput(gpio_h, LG_SET_PULL_NONE, READY_GPIO_PIN);
    if (rc < 0) {
        std::cerr << "spi_init: lgGpioClaimInput(READY) failed: "
                  << lguErrorText(rc) << "\n";
        return false;
    }

    rc = lgGpioClaimOutput(gpio_h, 0, CS_GPIO_PIN, 1);
    if (rc < 0) {
        std::cerr << "spi_init: lgGpioClaimOutput(CS) failed: "
                  << lguErrorText(rc) << "\n";
        return false;
    }

    spi_fd = open(device, O_RDWR);
    if (spi_fd < 0) {
        perror("spi_init: open");
        return false;
    }

    // STM SPI2 is configured CPOL=0, CPHA=1, so Pi must be SPI_MODE_1.
    // SPI_NO_CS because CS is manually controlled on GPIO25.
    uint8_t  mode  = SPI_MODE_1 | SPI_NO_CS;
    uint8_t  bits  = 8;
    uint32_t speed = speed_hz;

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("spi_init: SPI_IOC_WR_MODE");
        return false;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("spi_init: SPI_IOC_WR_BITS_PER_WORD");
        return false;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("spi_init: SPI_IOC_WR_MAX_SPEED_HZ");
        return false;
    }

    return true;
}

// =============================================================================
// write_telem_row
// =============================================================================
static void write_telem_row(std::ofstream& csv,
                            const TelemetryFrame& f,
                            uint32_t t0)
{
    csv << (f.timestamp_ms - t0) << ","
        << f.pos_cmd             << ","
        << f.pos_fbk             << ","
        << f.vel_cmd             << ","
        << f.vel_fbk             << ","
        << f.pos_err             << ","
        << f.i_q_fbk             << ","
        << f.samples_consumed    << "\n";
}

// =============================================================================
// spi_ready — active-low
// =============================================================================
bool spi_ready(void)
{
    if (gpio_h < 0) {
        return false;
    }

    return lgGpioRead(gpio_h, READY_GPIO_PIN) == 0;
}

// =============================================================================
// spi_send_block_header
// =============================================================================
bool spi_send_block_header(void)
{
    uint8_t tx[TRANSACTION_BYTES] = {};
    uint8_t rx[TRANSACTION_BYTES] = {};

    tx[0] = SPI2_OP_BLOCK_HDR;

    // Current STM ignores count payload, but keep bytes defined and CRC valid.
    tx[1] = 0;
    tx[2] = 0;
    tx[3] = crc8(tx, 3);

    return spi_transfer_raw(tx, rx, TRANSACTION_BYTES);
}

// =============================================================================
// spi_stream_block — sends up to count DATA frames starting at offset
// =============================================================================
size_t spi_stream_block(const std::vector<Sample>& profile,
                        size_t offset,
                        size_t count,
                        std::ofstream* csv,
                        uint32_t* telem_t0,
                        bool* t0_set,
                        int32_t* last_fbk,
                        bool send_header)
{
    if (send_header) {
        if (!spi_send_block_header()) {
            std::cerr << "spi: BLOCK_HDR failed\n";
            return 0;
        }

        usleep(200);
    }

    if (offset >= profile.size()) {
        return 0;
    }

    const size_t end = std::min(offset + count, profile.size());
    size_t sent = 0;

    for (size_t i = offset; i < end; i++) {
        const Sample& s = profile[i];

        uint8_t tx[TRANSACTION_BYTES] = {};
        uint8_t rx[TRANSACTION_BYTES] = {};

        int32_t pos = static_cast<int32_t>(s.pos);
        int32_t vel = static_cast<int32_t>(s.vel);

        tx[0] = SPI2_OP_DATA;
        std::memcpy(&tx[1], &pos, sizeof(int32_t));
        std::memcpy(&tx[5], &vel, sizeof(int32_t));
        tx[9] = crc8(tx, 9);

        if (!spi_transfer_raw(tx, rx, TRANSACTION_BYTES)) {
            std::cerr << "spi: DATA failed at sample " << i << "\n";
            return sent;
        }

        sent++;

        // MISO telemetry is currently disabled on STM.
        // Keep CSV handling guarded; it will become useful again when TX is restored.
        if (csv && telem_t0 && t0_set && last_fbk) {
            TelemetryFrame f;
            std::memcpy(&f, rx, sizeof(TelemetryFrame));

            if (!(*t0_set) && f.samples_consumed > 0) {
                *telem_t0 = f.timestamp_ms;
                *t0_set = true;
            }

            if (*t0_set && f.pos_fbk != *last_fbk) {
                write_telem_row(*csv, f, *telem_t0);
                *last_fbk = f.pos_fbk;
            }
        }
    }

    std::cout << "  streamed " << sent << " samples"
              << " [" << offset << "-" << (offset + sent - 1) << "]"
              << (send_header ? " initial/header" : " refill")
              << "\n";

    return sent;
}

// =============================================================================
// spi_stream_profile — full producer/consumer model with READY handshake
// =============================================================================
size_t spi_stream_profile(const std::vector<Sample>& profile,
                          std::ofstream* csv,
                          uint32_t* telem_t0,
                          bool* t0_set,
                          int32_t* last_fbk)
{
    size_t sent = 0;

    sent += spi_stream_block(profile,
                             sent,
                             INITIAL_FILL_FRAMES,
                             csv,
                             telem_t0,
                             t0_set,
                             last_fbk,
                             true);

    while (sent < profile.size()) {
        while (!spi_ready()) {
            usleep(50);
        }

        const size_t n = spi_stream_block(profile,
                                          sent,
                                          REFILL_FRAMES,
                                          csv,
                                          telem_t0,
                                          t0_set,
                                          last_fbk,
                                          false);

        if (n == 0) {
            break;
        }

        sent += n;

        while (spi_ready()) {
            usleep(50);
        }
    }

    return sent;
}

// =============================================================================
// spi_telem_poll
// =============================================================================
bool spi_telem_poll(TelemetryFrame* frame)
{
    if (frame == nullptr) {
        return false;
    }

    uint8_t tx[TRANSACTION_BYTES] = {};
    uint8_t rx[TRANSACTION_BYTES] = {};

    tx[0] = SPI2_OP_TELEM_REQ;
    tx[1] = crc8(tx, 1);

    if (!spi_transfer_raw(tx, rx, TRANSACTION_BYTES)) {
        return false;
    }

    std::memcpy(frame, rx, sizeof(TelemetryFrame));
    return true;
}

// =============================================================================
// spi_send_position — sends one int32 DATA frame
// =============================================================================
bool spi_send_position(int32_t counts)
{
    uint8_t tx[TRANSACTION_BYTES] = {};
    uint8_t rx[TRANSACTION_BYTES] = {};

    int32_t pos = counts;
    int32_t vel = 0;

    tx[0] = SPI2_OP_DATA;
    std::memcpy(&tx[1], &pos, sizeof(int32_t));
    std::memcpy(&tx[5], &vel, sizeof(int32_t));
    tx[9] = crc8(tx, 9);

    return spi_transfer_raw(tx, rx, TRANSACTION_BYTES);
}

// =============================================================================
// spi_send_open_loop
// =============================================================================
bool spi_send_open_loop(float v_mag, float d_theta)
{
    uint8_t tx[TRANSACTION_BYTES] = {};
    uint8_t rx[TRANSACTION_BYTES] = {};

    tx[0] = SPI2_OP_OPEN_LOOP;
    std::memcpy(&tx[1], &v_mag,   sizeof(float));
    std::memcpy(&tx[5], &d_theta, sizeof(float));
    tx[9] = crc8(tx, 9);

    return spi_transfer_raw(tx, rx, TRANSACTION_BYTES);
}

// =============================================================================
// spi_send_stop
// =============================================================================
bool spi_send_stop(void)
{
    uint8_t tx[TRANSACTION_BYTES] = {};
    uint8_t rx[TRANSACTION_BYTES] = {};

    tx[0] = SPI2_OP_STOP;
    tx[1] = crc8(tx, 1);

    return spi_transfer_raw(tx, rx, TRANSACTION_BYTES);
}

// =============================================================================
// spi_close
// =============================================================================
void spi_close()
{
    if (gpio_h >= 0) {
        lgGpioWrite(gpio_h, CS_GPIO_PIN, 1);
    }

    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }

    if (gpio_h >= 0) {
        lgGpiochipClose(gpio_h);
        gpio_h = -1;
    }
}