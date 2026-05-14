// spi.cpp — Pi-side SPI2 protocol implementation
// Raspberry Pi 5, spidev + lgpio (Pi 5 compatible), C++17
//
// PROTOCOL SUMMARY:
//   Every transaction is exactly 24 bytes — Pi drives clock, STM follows.
//   Pi sends opcode in byte 0, pads remainder with 0x00.
//   STM always returns latest TelemetryFrame on MISO during any transaction.
//   READY signal is a GPIO on BCM pin 25 connected to STM PC13 — not a SPI byte.

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

// ── Constants ─────────────────────────────────────────────────────────────────

static constexpr size_t TRANSACTION_BYTES = SPI2_TRANSACTION_BYTES;  // 24
static constexpr int    READY_GPIO_PIN    = 25;    // BCM pin connected to STM PC13
static constexpr int    READY_TIMEOUT_MS  = 500;   // max wait for READY signal

// ── File descriptors ──────────────────────────────────────────────────────────

static int spi_fd  = -1;
static int gpio_h  = -1;   // lgpio chip handle

// ── CRC8 ──────────────────────────────────────────────────────────────────────

static uint8_t crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) crc ^= data[i];
    return crc;
}

// ── Raw 24-byte SPI transfer ──────────────────────────────────────────────────
bool spi_transfer_raw(const uint8_t* tx, uint8_t* rx, size_t len)
{
    struct spi_ioc_transfer tr = {};
    tr.tx_buf        = (unsigned long)tx;
    tr.rx_buf        = (unsigned long)rx;
    tr.len           = len;
    tr.bits_per_word = 8;
    tr.cs_change     = 1;   // deassert CS after each transfer
    return ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) >= 0;
}

// ── Wait for READY GPIO ───────────────────────────────────────────────────────

static bool wait_for_ready(int timeout_ms = READY_TIMEOUT_MS)
{
    int elapsed = 0;
    while (elapsed < timeout_ms)
    {
        if (lgGpioRead(gpio_h, READY_GPIO_PIN) == 1) return true;
        usleep(1000);
        elapsed++;
    }
    std::cerr << "spi: timeout waiting for READY on GPIO " << READY_GPIO_PIN << "\n";
    return false;
}

// =============================================================================
// spi_init
// =============================================================================
bool spi_init(const char* device, uint32_t speed_hz)
{
    // open lgpio chip 0 — Pi 5 GPIO chip
    gpio_h = lgGpiochipOpen(0);
    if (gpio_h < 0) {
        std::cerr << "spi_init: lgGpiochipOpen failed: " << lguErrorText(gpio_h) << "\n";
        return false;
    }

    // configure READY pin as input — no pull, STM drives it
    int rc = lgGpioClaimInput(gpio_h, LG_SET_PULL_NONE, READY_GPIO_PIN);
    if (rc < 0) {
        std::cerr << "spi_init: lgGpioClaimInput failed: " << lguErrorText(rc) << "\n";
        return false;
    }

    // open spidev device
    spi_fd = open(device, O_RDWR);
    if (spi_fd < 0) { perror("spi_init: open"); return false; }

    uint8_t  mode  = SPI_MODE_0;
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
size_t spi_stream_block(const std::vector<Sample>& profile, size_t offset, size_t count)
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

    std::cout << "HDR rx: ";
    for (int i = 0; i < 8; i++)
        std::cout << std::hex << (int)hdr_rx[i] << " ";
        std::cout << std::dec << "\n";

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

        if (!spi_transfer_raw(tx, rx, TRANSACTION_BYTES)) 
        {
            std::cerr << "spi: DATA failed offset " << offset + i << "\n";
            return i;
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
// spi_stream_profile
// =============================================================================
bool spi_stream_profile(const std::vector<Sample>& profile, size_t block_size)
{
    size_t total    = profile.size();
    size_t n_blocks = (total + block_size - 1) / block_size;

    std::cout << "Streaming " << total << " samples in "
              << n_blocks << " blocks of " << block_size << "\n";

    for (size_t blk = 0; blk < n_blocks; blk++)
    {
        size_t start = blk * block_size;
        size_t end   = std::min(start + block_size, total);
        size_t n     = end - start;

        uint8_t hdr_tx[TRANSACTION_BYTES] = {};
        uint8_t hdr_rx[TRANSACTION_BYTES] = {};

        hdr_tx[0] = SPI2_OP_BLOCK_HDR;
        hdr_tx[1] = (n >> 8) & 0xFF;
        hdr_tx[2] =  n       & 0xFF;
        hdr_tx[3] = crc8(hdr_tx, 3);

        if (!spi_transfer_raw(hdr_tx, hdr_rx, TRANSACTION_BYTES)) {
            std::cerr << "spi: BLOCK_HDR failed at block " << blk << "\n";
            return false;
        }
        usleep(200);

        for (size_t i = 0; i < n; i++)
        {
            const Sample& s = profile[start + i];

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
                std::cerr << "spi: DATA failed block " << blk
                          << " sample " << i << "\n";
                return false;
            }
        }

        uint8_t ack_tx[TRANSACTION_BYTES] = {};
        uint8_t ack_rx[TRANSACTION_BYTES] = {};
        ack_tx[0] = SPI2_OP_READY_ACK;
        spi_transfer_raw(ack_tx, ack_rx, TRANSACTION_BYTES);

        std::cout << "  block " << blk + 1 << "/" << n_blocks
                  << "  samples " << start << "-" << end - 1 << "\n";
    }

    std::cout << "Stream complete.\n";
    return true;
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
// Returns true if STM PC13 READY signal is asserted low.
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
    if (spi_fd >= 0) { close(spi_fd); spi_fd = -1; }
    if (gpio_h >= 0) { lgGpiochipClose(gpio_h); gpio_h = -1; }
}