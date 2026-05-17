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
#include <csignal>
#include <cstdio>

static constexpr const char* DOCS_DIR   = "/home/jz/trajectory-streamer/docs";
static constexpr const char* SCRIPT_DIR = "/home/jz/trajectory-streamer/py-script";

static void sig_handler(int)
{
    spi_close();
    exit(0);
}

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

    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    while (true) {
        std::cout << "\nPress Enter to run move, q to quit: ";
        std::string input;
        std::getline(std::cin, input);
        if (input == "q") break;

        // ── 3. Precompute profile ─────────────────────────────────────────
        auto profile = compute_profile(start, target, vel, accel);
        std::cout << profile.size() << " samples ("
                  << profile.size() / 1000.0 << "s)\n";

        // ── 4. Clean previous run output ──────────────────────────────────
        std::remove((std::string(DOCS_DIR) + "/profile.csv").c_str());
        std::remove((std::string(DOCS_DIR) + "/telem.csv").c_str());
        std::remove((std::string(DOCS_DIR) + "/profile_plot.png").c_str());

        // ── 5. Write profile CSV ──────────────────────────────────────────
        {
            std::ofstream csv(std::string(DOCS_DIR) + "/profile.csv");
            csv << "sample,t,pos,vel\n";
            for (size_t i = 0; i < profile.size(); i++)
                csv << i << "," << i * 0.001 << ","
                    << profile[i].pos << "," << profile[i].vel << "\n";
        }

        // ── 6. Open telem CSV ─────────────────────────────────────────────
        std::ofstream telem_csv(std::string(DOCS_DIR) + "/telem.csv");
        telem_csv << "t,pos_cmd,pos_fbk,vel_fbk,pos_err,i_q_fbk,v_q_cmd,samples_consumed\n";

        uint32_t telem_t0 = 0;
        bool     t0_set   = false;
        int32_t  last_fbk = INT32_MIN;

        // ── 7. Stream initial block ───────────────────────────────────────
        std::cout << "Streaming...\n";
        auto   t0_stream = std::chrono::steady_clock::now();
        size_t sent      = spi_stream_block(profile, 0, 4096,
                                            &telem_csv, &telem_t0, &t0_set, &last_fbk);
        auto   t1_stream = std::chrono::steady_clock::now();
        std::cout << "Stream took "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         t1_stream - t0_stream).count()
                  << "ms\n";

        // ── 8. Telem + refill loop ────────────────────────────────────────
        std::cout << "Running...\n";
        TelemetryFrame frame = {};
        auto t_last = std::chrono::steady_clock::now();

        while (frame.samples_consumed < (uint32_t)profile.size()) {

            if (spi_telem_poll(&frame)) {
                if (!t0_set && frame.samples_consumed > 0) {
                    telem_t0 = frame.timestamp_ms;
                    t0_set   = true;
                }
                if (t0_set && frame.pos_fbk != last_fbk) {
                    telem_csv << (frame.timestamp_ms - telem_t0) << ","
                              << frame.pos_cmd                   << ","
                              << frame.pos_fbk                   << ","
                              << frame.vel_fbk                   << ","
                              << frame.pos_err                   << ","
                              << frame.i_q_fbk                   << ","
                              << frame.v_q_cmd                   << ","
                              << frame.samples_consumed          << "\n";
                    last_fbk = frame.pos_fbk;
                }
            }

            if (spi_ready() && sent < profile.size()) {
                std::cout << "READY triggered\n";
                size_t chunk = std::min(profile.size() - sent, (size_t)2048);
                sent += spi_stream_block(profile, sent, chunk,
                                         &telem_csv, &telem_t0, &t0_set, &last_fbk);
                std::cout << "Refilled — sent " << sent
                          << "/" << profile.size() << "\n";
            }

            auto t_now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    t_now - t_last).count() > 500) {
                std::cout << "consumed=" << frame.samples_consumed
                          << "/" << profile.size()
                          << "  pos_fbk=" << frame.pos_fbk
                          << "  pos_err=" << frame.pos_err
                          << "  i_q_fbk=" << frame.i_q_fbk
                          << "  v_q_cmd=" << frame.v_q_cmd
                          << "  ts="      << frame.timestamp_ms
                          << "\n";
                t_last = t_now;
            }
        }

        telem_csv.close();
        std::cout << "Move complete. Written to docs/telem.csv\n";

        // ── 9. Plot ───────────────────────────────────────────────────────
        int ret = system((std::string("python3 ") + SCRIPT_DIR + "/plotprof.py " +
                          DOCS_DIR + "/profile.csv " +
                          DOCS_DIR + "/telem.csv").c_str());
        std::cout << "Plot exit code: " << ret << "\n";
    }

    spi_close();
    return 0;
}