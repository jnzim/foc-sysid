#pragma once
#include <cstdint>
#include <vector>
#include <fstream>
#include "profile.hpp"
#include "protocol.h"

// open the spidev device (e.g. "/dev/spidev0.0") at the given clock speed
bool spi_init(const char* device, uint32_t speed_hz);

// stream a block of trajectory samples to the STM ring buffer
// send_header=true  → first block of a new move (sends BLOCK_HDR, resets STM ring)
// send_header=false → refill block (DATA packets only, no ring reset, no telem corruption)
size_t spi_stream_block(const std::vector<Sample>& profile,
                        size_t offset, size_t count,
                        std::ofstream* csv      = nullptr,
                        uint32_t* telem_t0      = nullptr,
                        bool* t0_set            = nullptr,
                        int32_t* last_fbk       = nullptr,
                        bool send_header        = true);

// send a single position setpoint — used for testing before full streaming
bool spi_send_position(int32_t counts);

// command STM to STATE_OPEN_LOOP
// v_mag:   voltage magnitude in volts (e.g. 1.5f at 12V bus)
// d_theta: angle increment per SysTick tick — 1Hz = 2π/1000 = 0.00628f
bool spi_send_open_loop(float v_mag, float d_theta);

// command STM to STATE_IDLE from any running state
bool spi_send_stop(void);

// raw SPI transfer
bool spi_transfer_raw(const uint8_t* tx, uint8_t* rx, size_t len);

// send TELEM_REQ opcode, receive latest TelemetryFrame from STM on MISO
bool spi_telem_poll(TelemetryFrame* frame);

// returns true if PC13 READY signal is asserted low — STM ring buffer needs refill
bool spi_ready(void);

// close the spidev file descriptor
void spi_close();