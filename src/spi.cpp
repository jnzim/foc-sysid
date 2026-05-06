#include "spi.hpp"
#include "profile.hpp"
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <iostream>
#include <algorithm>

static int spi_fd = -1;

// ── Internal helpers ──────────────────────────────────────────────────────────

static uint8_t crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) crc ^= data[i];
    return crc;
}

// ── Public API ────────────────────────────────────────────────────────────────

bool spi_init(const char* device, uint32_t speed_hz)
{
    spi_fd = open(device, O_RDWR);
    if (spi_fd < 0) { perror("spi_init: open"); return false; }

    uint8_t  mode  = SPI_MODE_1;
    uint8_t  bits  = 8;
    uint32_t speed = speed_hz;

    ioctl(spi_fd, SPI_IOC_WR_MODE,          &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ,  &speed);
    return true;
}

bool spi_transfer_raw(const uint8_t* tx, uint8_t* rx, size_t len)
{
    struct spi_ioc_transfer tr = {};
    tr.tx_buf        = (unsigned long)tx;
    tr.rx_buf        = (unsigned long)rx;
    tr.len           = len;
    tr.speed_hz      = 1000000;
    tr.bits_per_word = 8;
    return ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) >= 0;
}

static bool wait_for_ready(int timeout_ms = 500)
{
    uint8_t tx = 0x00;
    uint8_t rx = 0x00;
    int elapsed = 0;
    while (elapsed < timeout_ms)
    {
        spi_transfer_raw(&tx, &rx, 1);
        if (rx == 0x05) return true;
        usleep(1000);
        elapsed++;
    }
    std::cerr << "spi: timeout waiting for STM32 READY\n";
    return false;
}

bool spi_stream_profile(const std::vector<Sample>& profile, size_t block_size)
{
    size_t total_samples = profile.size();
    size_t total_blocks  = (total_samples + block_size - 1) / block_size;

    std::cout << "Streaming " << total_samples << " samples ("
              << total_samples / 1000.0 << "s) in "
              << total_blocks << " blocks\n";

    for (size_t block_idx = 0; block_idx < total_blocks; block_idx++)
    {
        size_t sample_start = block_idx * block_size;
        size_t sample_end   = std::min(sample_start + block_size, total_samples);
        size_t n_samples    = sample_end - sample_start;

        // ── Header packet (9 bytes) ───────────────────────────────────────
        //   [0]    0xAB        sync
        //   [1]    0x03        msg type: block header
        //   [2-3]  uint16_t    block index       (big-endian)
        //   [4-5]  uint16_t    samples this block (big-endian)
        //   [6-7]  uint16_t    total blocks       (big-endian)
        //   [8]    CRC8
        uint8_t header[9] = {};
        header[0] = 0xAB;
        header[1] = 0x03;
        header[2] = (block_idx    >> 8) & 0xFF;
        header[3] =  block_idx          & 0xFF;
        header[4] = (n_samples    >> 8) & 0xFF;
        header[5] =  n_samples          & 0xFF;
        header[6] = (total_blocks >> 8) & 0xFF;
        header[7] =  total_blocks       & 0xFF;
        header[8] = crc8(header, 8);

        uint8_t rx9[9] = {};
        if (!spi_transfer_raw(header, rx9, sizeof(header))) {
            std::cerr << "spi: header failed at block " << block_idx << "\n";
            return false;
        }

        if (!wait_for_ready()) return false;

        // ── Data packet ───────────────────────────────────────────────────
        //   [0]              0xAB           sync
        //   [1]              0x04           msg type: block data
        //   [2..N*8+1]       Sample × N     pos(int32) + vel(int32), big-endian
        //   [N*8+2]          CRC8
        size_t data_len = 2 + (n_samples * 8) + 1;  // 8 bytes per sample
        std::vector<uint8_t> pkt(data_len, 0);

        pkt[0] = 0xAB;
        pkt[1] = 0x04;

        for (size_t i = 0; i < n_samples; i++)
        {
            const Sample& s = profile[sample_start + i];
            size_t off = 2 + i * 8;

            // Position — big-endian int32_t
            pkt[off + 0] = (s.pos >> 24) & 0xFF;
            pkt[off + 1] = (s.pos >> 16) & 0xFF;
            pkt[off + 2] = (s.pos >>  8) & 0xFF;
            pkt[off + 3] =  s.pos        & 0xFF;

            // Velocity — big-endian int32_t
            pkt[off + 4] = (s.vel >> 24) & 0xFF;
            pkt[off + 5] = (s.vel >> 16) & 0xFF;
            pkt[off + 6] = (s.vel >>  8) & 0xFF;
            pkt[off + 7] =  s.vel        & 0xFF;
        }

        pkt[data_len - 1] = crc8(pkt.data(), data_len - 1);

        std::vector<uint8_t> rx(data_len, 0);
        if (!spi_transfer_raw(pkt.data(), rx.data(), data_len)) {
            std::cerr << "spi: data failed at block " << block_idx << "\n";
            return false;
        }

        if (block_idx < total_blocks - 1) {
            if (!wait_for_ready()) return false;
        }

        std::cout << "  block " << block_idx + 1 << "/" << total_blocks
                  << "  samples " << sample_start << "-" << sample_end - 1
                  << "\n";
    }

    std::cout << "Stream complete.\n";
    return true;
}

bool spi_send_position(int32_t counts)
{
    uint8_t pkt[8] = {};
    pkt[0] = 0xAB;
    pkt[1] = 0x01;
    pkt[2] = (counts >> 24) & 0xFF;
    pkt[3] = (counts >> 16) & 0xFF;
    pkt[4] = (counts >>  8) & 0xFF;
    pkt[5] =  counts        & 0xFF;
    pkt[6] = 0x00;
    pkt[7] = crc8(pkt, 7);

    uint8_t rx[8] = {};
    return spi_transfer_raw(pkt, rx, sizeof(pkt));
}

void spi_close()
{
    if (spi_fd >= 0) close(spi_fd);
}