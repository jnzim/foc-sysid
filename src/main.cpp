#include "config.hpp"
#include "profile.hpp"
#include "spi.hpp"
#include "protocol.h"
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <climits>
#include <chrono>

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
        // add this:
        auto t_stream_start = std::chrono::steady_clock::now();
        size_t sent = spi_stream_block(profile, 0, 4096);
        auto t_stream_end = std::chrono::steady_clock::now();
        std::cout << "Stream took " 
          << std::chrono::duration_cast<std::chrono::milliseconds>(t_stream_end - t_stream_start).count()
          << "ms\n";

        // ── 6. Telem + refill loop ────────────────────────────────────────
        std::cout << "Running...\n";
        TelemetryFrame frame  = {};
        std::ofstream telem_csv("telem.csv");
        telem_csv << "t,pos_cmd,pos_fbk,vel_fbk,samples_consumed\n";
        uint32_t t0       = 0;
        bool     t0_set   = false;
        int32_t  last_fbk = INT32_MIN;

        while (frame.samples_consumed < (uint32_t)profile.size() -1 ) {
            if (spi_telem_poll(&frame)) {
                if (!t0_set && frame.samples_consumed > 0) {
                    t0     = frame.timestamp_ms;
                    t0_set = true;
                }
                if (t0_set && frame.pos_fbk != last_fbk) {
                    telem_csv << (frame.timestamp_ms - t0) << ","
                              << frame.pos_cmd             << ","
                              << frame.pos_fbk             << ","
                              << frame.vel_fbk             << ","
                              << frame.samples_consumed    << "\n";
                    last_fbk = frame.pos_fbk;
                }
            }
            if (spi_ready() && sent < profile.size()) {
                size_t chunk = std::min(profile.size() - sent, (size_t)2048);
                sent += spi_stream_block(profile, sent, chunk);
                std::cout << "Refilled — sent " << sent
                          << "/" << profile.size() << "\n";
            }

            static auto t_last = std::chrono::steady_clock::now();
            auto t_now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_last).count() > 500) {
                std::cout << "consumed=" << frame.samples_consumed
                          << "/" << profile.size()
                          << "  pos_fbk=" << frame.pos_fbk
                          << "  ts=" << frame.timestamp_ms
                          << "\n";
                t_last = t_now;
            }
        }
        telem_csv.close();
        std::cout << "Move complete. Written to telem.csv\n";

        // ── 7. Plot trajectory ────────────────────────────────────────────
        int ret = system("python3 /home/jz/trajectory-streamer/py-script/plotprof.py "
                 "/home/jz/trajectory-streamer/build/profile.csv "
                 "/home/jz/trajectory-streamer/build/telem.csv");
        std::cout << "Plot exit code: " << ret << "\n";
    }

    spi_close();
    return 0;
}