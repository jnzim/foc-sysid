#pragma once
#include <cstdint>
#include <vector>
#include "profile.hpp"

// open the spidev device (e.g. "/dev/spidev0.0") at the given clock speed
bool spi_init(const char* device, uint32_t speed_hz);

// stream a precomputed trajectory profile to the STM in 2048-sample blocks
// follows the SPI2 protocol spec — BLOCK_HDR, DATA packets, READY handshake
bool spi_stream_profile(const std::vector<Sample>& profile, size_t block_size = 256);

// send a single position setpoint — used for testing before full streaming
bool spi_send_position(int32_t counts);

// raw SPI transfer — Pi drives clock, tx and rx are both len bytes
// STM always returns latest TelemetryFrame on MISO during any transaction
bool spi_transfer_raw(const uint8_t* tx, uint8_t* rx, size_t len);

// close the spidev file descriptor
void spi_close();