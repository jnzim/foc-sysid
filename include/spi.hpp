#pragma once
#include <cstdint>
#include <vector>
#include <fstream>
#include "profile.hpp"
#include "protocol.h"

// open the spidev device (e.g. "/dev/spidev0.0") at the given clock speed
bool spi_init(const char* device, uint32_t speed_hz);



size_t spi_stream_block(const std::vector<Sample>& profile,
                        size_t offset, size_t count,
                        std::ofstream* csv = nullptr,
                        uint32_t* telem_t0 = nullptr,
                        bool* t0_set = nullptr,
                        int32_t* last_fbk = nullptr);

// send a single position setpoint — used for testing before full streaming
bool spi_send_position(int32_t counts);

// raw SPI transfer — Pi drives clock, tx and rx are both len bytes
// STM always returns latest TelemetryFrame on MISO during any transaction
bool spi_transfer_raw(const uint8_t* tx, uint8_t* rx, size_t len);

// send TELEM_REQ opcode, receive latest TelemetryFrame from STM on MISO
bool spi_telem_poll(TelemetryFrame* frame);

// returns true if PC13 READY signal is asserted low — STM ring buffer needs refill
bool spi_ready(void);

// close the spidev file descriptor and terminate pigpio
void spi_close();