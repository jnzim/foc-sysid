#pragma once
#include <cstdint>
#include <cmath>

struct Config
{
    double counts_per_rev = 8192.0;  // encoder counts per motor revolution
    double mm_per_rev     = 2.0;     // lead screw pitch in mm/rev

    double  counts_per_mm() const { return counts_per_rev / mm_per_rev; }
    int32_t mm_to_counts(double mm) const 
    {
        return static_cast<int32_t>(std::round(mm * counts_per_mm()));
    }
};

inline const Config MACHINE;