#include "config.hpp"
#include "profile.hpp"
#include "spi.hpp"
#include "protocol.h"
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

int main()
{
    // ── 1. Init SPI ───────────────────────────────────────────────────────
    if (!spi_init("/dev/spidev0.0", 1000000)) {
        std::cerr << "SPI init failed\n";
        return 1;
    }

    // ── 2. Convert mm inputs to encoder counts ────────────────────────────
    int32_t start  = MACHINE.mm_to_counts(0.0);
    int32_t target = MACHINE.mm_to_counts(10.0);
    int32_t vel    = MACHINE.mm_to_counts(8.0);
    int32_t accel  = MACHINE.mm_to_counts(8.0);

    while (true) {
        std::cout << "\nPress Enter to run move, q to quit: ";
        std::string input;
        std::getline(std::cin, input);
        if (input == "q") break;

        // ── 3. Precompute profile ─────────────────────────────────────────
        auto profile = compute_profile(start, target, vel, accel);
        std::cout << profile.size() << " samples ("
                  << profile.size() / 1000.0 << "s)\n";

        // ── 4. Write CSV ──────────────────────────────────────────────────
        std::ofstream csv("profile.csv");
        csv << "sample,t,pos,vel\n";
        for (size_t i = 0; i < profile.size(); i++)
            csv << i << "," << i * 0.001 << ","
                << profile[i].pos << "," << profile[i].vel << "\n";
        csv.close();
        std::cout << "Written to profile.csv\n";

        // ── 5. Fill buffer ────────────────────────────────────────────────
        std::cout << "Streaming...\n";
        size_t sent = spi_stream_block(profile, 0, 4096);

        // ── 6. Telem + refill loop ────────────────────────────────────────
        std::cout << "Running...\n";
        TelemetryFrame frame = {};
        std::ofstream telem_csv("telem.csv");
        telem_csv << "t,pos_cmd,samples_consumed\n";

        while (frame.samples_consumed < (uint32_t)profile.size()) {
            if (spi_telem_poll(&frame)) {
                telem_csv << frame.timestamp_ms     << ","
                          << frame.pos_cmd          << ","
                          << frame.samples_consumed << "\n";
                std::cout << "pos_cmd: "    << frame.pos_cmd
                          << "  consumed: " << frame.samples_consumed
                          << "  t: "        << frame.timestamp_ms << "\n";
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

       // ── 7. Plot trajectory ─────────────────────────────────────────────────
        std::cout << "Plotting...\n";
        system("python3 /home/jz/py-script/plotprof.py "
        "/home/jz/MotionController/profile.csv "
        "/home/jz/MotionController/telem.csv");

    }

    spi_close();
    return 0;
}