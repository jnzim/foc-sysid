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

    // ── 5. Fill buffer to 4096 — motor starts on first sample ────────────
    std::cout << "Streaming initial block...\n";
    size_t sent = spi_stream_block(profile, 0, 4096);

    // ── 6. Telem + refill loop until full profile echoed back ─────────────
    std::cout << "Running...\n";
    TelemetryFrame frame;
    std::ofstream telem_csv("telem.csv");
    telem_csv << "t,pos_cmd\n";

    for (size_t i = 0; i < profile.size(); i++) {
        if (spi_telem_poll(&frame)) {
            telem_csv << frame.timestamp_ms << ","
                      << frame.pos_cmd      << "\n";
            std::cout << "pos_cmd: " << frame.pos_cmd
                      << "  t: "     << frame.timestamp_ms << "\n";
        }

        if (spi_ready() && sent < profile.size()) {
            size_t chunk = std::min(profile.size() - sent, (size_t)2048);
            sent += spi_stream_block(profile, sent, chunk);
            std::cout << "Refilled — sent " << sent
                      << "/" << profile.size() << "\n";
        }

        usleep(1000);
    }

    telem_csv.close();
    std::cout << "Move complete. Written to telem.csv\n";
    spi_close();
    return 0;
}