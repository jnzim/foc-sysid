#define DEBUG_LOGGING  // comment this out for release

#include "config.hpp"
#include "profile.hpp"
#include "spi.hpp"
#include <iostream>
#include <fstream>



int main()
{
    // ── 1. Convert mm inputs to encoder counts ────────────────────────────
    int32_t start  = MACHINE.mm_to_counts(0.0);
    int32_t target = MACHINE.mm_to_counts(5.0);
    int32_t vel   = MACHINE.mm_to_counts(1.0);
    int32_t accel = MACHINE.mm_to_counts(100.0);

    // ── 2. Precompute full profile ────────────────────────────────────────
    auto profile = compute_profile(start, target, vel, accel);
    std::cout << profile.size() << " samples ("
              << profile.size() / 1000.0 << "s)\n";


#ifdef DEBUG_LOGGING

    // ── 3. Write CSV ──────────────────────────────────────────────────────
    std::ofstream csv("profile.csv");
    csv << "sample,t,pos,vel\n";
    for (size_t i = 0; i < profile.size(); i++)
        csv << i << ","
            << i * 0.001 << ","
            << profile[i].pos << ","
            << profile[i].vel << "\n";
    csv.close();
    std::cout << "Written to profile.csv\n";

#endif
    return 0;
}

