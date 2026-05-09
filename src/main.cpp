#define DEBUG_LOGGING
#include "config.hpp"
#include "profile.hpp"
#include "spi.hpp"
#include "protocol.h"
#include <iostream>
#include <fstream>
#include <unistd.h>

int main()
{
    // ── 1. Convert mm inputs to encoder counts ────────────────────────────
    int32_t start  = MACHINE.mm_to_counts(0.0);
    int32_t target = MACHINE.mm_to_counts(5.0);
    int32_t vel    = MACHINE.mm_to_counts(1.0);
    int32_t accel  = MACHINE.mm_to_counts(100.0);

    // ── 2. Precompute full profile ────────────────────────────────────────
    auto profile = compute_profile(start, target, vel, accel);
    std::cout << profile.size() << " samples ("
              << profile.size() / 1000.0 << "s)\n";

#ifdef DEBUG_LOGGING
    // ── 3. Write CSV ──────────────────────────────────────────────────────
    std::ofstream csv("profile.csv");
    csv << "sample,t,pos,vel\n";
    for (size_t i = 0; i < profile.size(); i++)
        csv << i << "," << i * 0.001 << ","
            << profile[i].pos << "," << profile[i].vel << "\n";
    csv.close();
    std::cout << "Written to profile.csv\n";
#endif

    // ── 4. Init SPI ───────────────────────────────────────────────────────
    if (!spi_init("/dev/spidev0.0", 1000000)) {
        std::cerr << "SPI init failed\n";
        return 1;
    }

    // ── 5. Stream profile to STM ──────────────────────────────────────────
    std::cout << "Streaming profile...\n";
    spi_stream_profile(profile, 256);
    std::cout << "Stream complete\n";

    // ── 6. Poll telem — verify STM received data ──────────────────────────
    std::cout << "Polling telem...\n";
    TelemetryFrame frame;
    for (int i = 0; i < 10; i++) {
        if (spi_telem_poll(&frame)) {
            std::cout << "pos_cmd: 0x" << std::hex << frame.pos_cmd
                      << "  pos_fbk: 0x" << frame.pos_fbk
                      << "  state: " << std::dec << (int)frame.drive_state << "\n";
        }
        usleep(1000);
    }

    spi_close();
    return 0;
}