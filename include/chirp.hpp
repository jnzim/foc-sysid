#pragma once
#include <cstdint>
#include <vector>
#include "profile.hpp"

// Generate a linear chirp position profile
// Sweeps from f_start to f_end Hz over duration seconds
// Amplitude in encoder counts, sample rate 1kHz
std::vector<Sample> compute_chirp(int32_t amplitude_counts,
                                   double f_start_hz,
                                   double f_end_hz,
                                   double duration_s);