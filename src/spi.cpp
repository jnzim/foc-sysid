// spi.cpp — Pi-side SPI2 protocol implementation
// Raspberry Pi 5, spidev + lgpio (Pi 5 compatible), C++17
//
// CS is controlled manually via GPIO8 (CE0) using lgpio.
// spidev automatic CS is disabled (cs_change = 0, no kernel CS toggling).
// This guarantees CS pulses between every 24-byte transaction.

#include "spi.hpp"
#include "protocol.h"
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <lgpio.h>
#include <iostream>
#include <algorithm>
#include <fstream>

static constexpr size_t TRANSACTION_BYTES = SPI2_TRANSACTION_BYTES;
static constexpr int    READY_GPIO_PIN    = 25;
static constexpr int    CS_GPIO_PIN       = 7;   // GPIO7, manual CS
static constexpr int    READY_TIMEOUT_MS  = 500;

static int spi_fd  = -1;
static int gpio_h  = -1;

static uint8_t crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) crc ^= data[i];
    return crc;
}

// ── Raw 24-byte SPI transfer — manual CS via GPIO ────────────────────────────
bool spi_transfer_raw(const uint8_t* tx, uint8_t* rx, size_t len)
{
    lgGpioWrite(gpio_h, CS_GPIO_PIN, 0);    // assert CS low

    struct spi_ioc_transfer tr = {};
    tr.tx_buf        = (unsigned long)tx;
    tr.rx_buf        = (unsigned long)rx;
    tr.len           = len;
    tr.bits_per_word = 8;
    tr.cs_change     = 0;                   // we control CS manually

    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);

    lgGpioWrite(gpio_h, CS_GPIO_PIN, 1);    // deassert CS high

    return ret >= 0;
}

// =============================================================================
// spi_init
// =============================================================================
bool spi_init(const char* device, uint32_t speed_hz)
{
    gpio_h = lgGpiochipOpen(0);
    if (gpio_h < 0) {
        std::cerr << "spi_init: lgGpiochipOpen failed: " << lguErrorText(gpio_h) << "\n";
        return false;
    }

    // READY input — Pi reads PC13 from STM
    int rc = lgGpioClaimInput(gpio_h, LG_SET_PULL_NONE, READY_GPIO_PIN);
    if (rc < 0) {
        std::cerr << "spi_init: lgGpioClaimInput(READY) failed: " << lguErrorText(rc) << "\n";
        return false;
    }

    // CS output — start deasserted (high)
    rc = lgGpioClaimOutput(gpio_h, 0, CS_GPIO_PIN, 1);
    if (rc < 0) {
        std::cerr << "spi_init: lgGpioClaimOutput(CS) failed: " << lguErrorText(rc) << "\n";
        return false;
    }

    spi_fd = open(device, O_RDWR);
    if (spi_fd < 0) { perror("spi_init: open"); return false; }

    uint8_t  mode  = SPI_MODE_0 | SPI_NO_CS;  // disable kernel CS — we drive it manually
    uint8_t  bits  = 8;
    uint32_t speed = speed_hz;

    ioctl(spi_fd, SPI_IOC_WR_MODE,          &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ,  &speed);

    return true;
}

// =============================================================================
// spi_stream_block
// =============================================================================
size_t spi_stream_block(const std::vector<Sample>& profile,
                        size_t offset, size_t count,
                        std::ofstream* csv,
                        uint32_t* telem_t0, bool* t0_set, int32_t* last_fbk)
{
    size_t end = std::min(offset + count, profile.size());
    size_t n   = end - offset;
    if (n == 0) return 0;

    // ── BLOCK_HDR ─────────────────────────────────────────────────────────
    uint8_t hdr_tx[TRANSACTION_BYTES] = {};
    uint8_t hdr_rx[TRANSACTION_BYTES] = {};
    hdr_tx[0] = SPI2_OP_BLOCK_HDR;
    hdr_tx[1] = (n >> 8) & 0xFF;
    hdr_tx[2] =  n       & 0xFF;
    hdr_tx[3] = crc8(hdr_tx, 3);

    if (!spi_transfer_raw(hdr_tx, hdr_rx, TRANSACTION_BYTES)) {
        std::cerr << "spi: BLOCK_HDR failed at offset " << offset << "\n";
        return 0;
    }
    usleep(200);

    // ── DATA packets ──────────────────────────────────────────────────────
    for (size_t i = 0; i < n; i++) {
        const Sample& s = profile[offset + i];

        uint8_t tx[TRANSACTION_BYTES] = {};
        uint8_t rx[TRANSACTION_BYTES] = {};

        tx[0] = SPI2_OP_DATA;
        tx[1] =  s.pos        & 0xFF;
        tx[2] = (s.pos >>  8) & 0xFF;
        tx[3] = (s.pos >> 16) & 0xFF;
        tx[4] = (s.pos >> 24) & 0xFF;
        tx[5] =  s.vel        & 0xFF;
        tx[6] = (s.vel >>  8) & 0xFF;
        tx[7] = (s.vel >> 16) & 0xFF;
        tx[8] = (s.vel >> 24) & 0xFF;
        tx[9] = crc8(tx, 9);

        if (!spi_transfer_raw(tx, rx, TRANSACTION_BYTES)) {
            std::cerr << "spi: DATA failed offset " << offset + i << "\n";
            return i;
        }

        // ── Parse MISO telem during streaming ─────────────────────────────
        if (csv && telem_t0 && t0_set && last_fbk) {
            TelemetryFrame f;
            memcpy(&f, rx, sizeof(TelemetryFrame));
            if (!(*t0_set) && f.samples_consumed > 0) {
                *telem_t0 = f.timestamp_ms;
                *t0_set   = true;
            }
            if (*t0_set && f.pos_fbk != *last_fbk) {
                *csv << (f.timestamp_ms - *telem_t0) << ","
                     << f.pos_cmd                    << ","
                     << f.pos_fbk                    << ","
                     << f.vel_fbk                    << ","
                     << f.samples_consumed           << "\n";
                *last_fbk = f.pos_fbk;
            }
        }

        usleep(200);
    }

    // ── READY_ACK ─────────────────────────────────────────────────────────
    uint8_t ack_tx[TRANSACTION_BYTES] = {};
    uint8_t ack_rx[TRANSACTION_BYTES] = {};
    ack_tx[0] = SPI2_OP_READY_ACK;
    spi_transfer_raw(ack_tx, ack_rx, TRANSACTION_BYTES);

    std::cout << "  streamed " << n << " samples"
              << " [" << offset << "-" << end - 1 << "]\n";

    return n;
}

// =============================================================================
// spi_telem_poll
// =============================================================================
bool spi_telem_poll(TelemetryFrame* frame)
{
    uint8_t tx[TRANSACTION_BYTES] = {};
    uint8_t rx[TRANSACTION_BYTES] = {};
    tx[0] = SPI2_OP_TELEM_REQ;
    if (!spi_transfer_raw(tx, rx, TRANSACTION_BYTES)) return false;
    memcpy(frame, rx, sizeof(TelemetryFrame));
    return true;
}

// =============================================================================
// spi_send_position
// =============================================================================
bool spi_send_position(int32_t counts)
{
    uint8_t tx[TRANSACTION_BYTES] = {};
    uint8_t rx[TRANSACTION_BYTES] = {};
    tx[0] = SPI2_OP_DATA;
    tx[1] =  counts        & 0xFF;
    tx[2] = (counts >>  8) & 0xFF;
    tx[3] = (counts >> 16) & 0xFF;
    tx[4] = (counts >> 24) & 0xFF;
    tx[9] = crc8(tx, 9);
    return spi_transfer_raw(tx, rx, TRANSACTION_BYTES);
}

// =============================================================================
// spi_ready
// =============================================================================
bool spi_ready(void)
{
    return lgGpioRead(gpio_h, READY_GPIO_PIN) == 0;
}

// =============================================================================
// spi_close
// =============================================================================
void spi_close()
{
    lgGpioWrite(gpio_h, CS_GPIO_PIN, 1);    // deassert CS on close
    if (spi_fd >= 0) { close(spi_fd); spi_fd = -1; }
    if (gpio_h >= 0) { lgGpiochipClose(gpio_h); gpio_h = -1; }
}