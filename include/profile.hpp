#pragma once
#include <vector>
#include <cstdint>

struct Sample
{
    int32_t pos;  // encoder counts
    int32_t vel;  // encoder counts/sec
};

std::vector<Sample> compute_profile(int32_t start_cnt,
                                    int32_t target_cnt,
                                    int32_t vel_cnt,
                                    int32_t accel_cnt,
                                    double  dt = 0.001);