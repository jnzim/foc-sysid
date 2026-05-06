#pragma once
#include <cstdint>
#include <vector>
#include "profile.hpp"

bool spi_init(const char* device, uint32_t speed_hz);
bool spi_stream_profile(const std::vector<Sample>& profile, size_t block_size = 256);
bool spi_send_position(int32_t counts);
bool spi_transfer_raw(const uint8_t* tx, uint8_t* rx, size_t len);
void spi_close();