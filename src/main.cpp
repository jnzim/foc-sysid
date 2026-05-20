#include "config.hpp"
#include "profile.hpp"
#include "chirp.hpp"
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
#include <cmath>

static constexpr const char* DOCS_DIR   = "/home/jz/trajectory-streamer/docs";
static constexpr const char* SCRIPT_DIR = "/home/jz/trajectory-streamer/py-script";

static void sig_handler(int)
{
    spi_send_stop();
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

    // ── 2. Trapezoidal move parameters ───────────────────────────────────
    int32_t start  = MACHINE.mm_to_counts(0.0);
    int32_t target = MACHINE.mm_to_counts(22.0);
    int32_t vel    = MACHINE.mm_to_counts(8.0);
    int32_t accel  = MACHINE.mm_to_counts(8.0);

    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    while (true) {
        std::cout << "\nEnter to run move, c=chirp, o=open loop, s=stop, q=quit: ";
        std::string input;
        std::getline(std::cin, input);

        if (input == "q") break;

        // ── Open loop — spin motor without feedback ───────────────────────
        // v_mag:   voltage in volts — keep low at 12V bus (1.0-1.5V)
        // d_theta: angle per SysTick tick — 1Hz = 2π/1000 = 0.00628f
        if (input == "o") {
            float v_mag   = 1.5f;
            float d_theta = 2.0f * M_PI / 1000.0f;   // 1Hz electrical
            std::cout << "Open loop: v_mag=" << v_mag
                      << "V  f_elec=1Hz  d_theta=" << d_theta << "\n";
            if (!spi_send_open_loop(v_mag, d_theta))
                std::cerr << "spi_send_open_loop failed\n";
            continue;
        }

        // ── Stop — return STM to STATE_IDLE ──────────────────────────────
        if (input == "s") {
            std::cout << "Sending stop...\n";
            if (!spi_send_stop())
                std::cerr << "spi_send_stop failed\n";
            continue;
        }

        bool chirp_mode = (input == "c");

        // ── 3. Precompute profile ─────────────────────────────────────────
        std::vector<Sample> profile;
        if (chirp_mode) {
            profile = compute_chirp(200, 0.1, 250.0, 10.0);
            std::cout << "Chirp: 0.1→250Hz, ±200 counts, 10s\n";
        } else {
            profile = compute_profile(start, target, vel, accel);
        }
        std::cout << profile.size() << " samples ("
                  << profile.size() / 1000.0 << "s)\n";

        // ── 4. Clean previous run output ──────────────────────────────────
        std::string prof_csv    = chirp_mode ? "/chirp.csv"       : "/profile.csv";
        std::string telem_f     = chirp_mode ? "/chirp_telem.csv" : "/telem.csv";
        std::string plot_out    = chirp_mode ? "/bode.png"        : "/profile_plot.png";
        std::string plot_script = chirp_mode ? "/plotbode.py"     : "/plotprof.py";

        std::remove((std::string(DOCS_DIR) + prof_csv).c_str());
        std::remove((std::string(DOCS_DIR) + telem_f).c_str());
        std::remove((std::string(DOCS_DIR) + plot_out).c_str());

        // ── 5. Write profile CSV ──────────────────────────────────────────
        {
            std::ofstream csv(std::string(DOCS_DIR) + prof_csv);
            csv << "sample,t,pos,vel\n";
            for (size_t i = 0; i < profile.size(); i++)
                csv << i << "," << i * 0.001 << ","
                    << profile[i].pos << "," << profile[i].vel << "\n";
        }

        // ── 6. Open telem CSV ─────────────────────────────────────────────
        std::ofstream telem_csv(std::string(DOCS_DIR) + telem_f);
        telem_csv << "t,pos_cmd,pos_fbk,vel_fbk,pos_err,i_q_fbk,v_q_cmd,samples_consumed\n";

        uint32_t telem_t0 = 0;
        bool     t0_set   = false;
        int32_t  last_fbk = INT32_MIN;

        // ── 7. Stream initial block ───────────────────────────────────────
        std::cout << "Streaming...\n";
        auto t0_stream = std::chrono::steady_clock::now();
        size_t sent    = spi_stream_block(profile, 0, 4096,
                                          &telem_csv, &telem_t0, &t0_set, &last_fbk,
                                          true);
        auto t1_stream = std::chrono::steady_clock::now();
        std::cout << "Stream took "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         t1_stream - t0_stream).count()
                  << "ms\n";

        // ── 8. Telem + refill loop ────────────────────────────────────────
        std::cout << (chirp_mode ? "Running chirp...\n" : "Running...\n");
        TelemetryFrame frame  = {};
        auto t_last           = std::chrono::steady_clock::now();
        auto t_run_start      = std::chrono::steady_clock::now();

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

            usleep(500);

            if (spi_ready() && sent < profile.size()) {
                std::cout << "READY triggered — refilling\n";
                size_t chunk = std::min(profile.size() - sent, (size_t)2048);
                sent += spi_stream_block(profile, sent, chunk,
                                         &telem_csv, &telem_t0, &t0_set, &last_fbk,
                                         false);
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
                          << "  ts="      << frame.timestamp_ms
                          << "\n";
                t_last = t_now;
            }
        }

        auto t_run_end = std::chrono::steady_clock::now();
        std::cout << "Complete in "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         t_run_end - t_run_start).count()
                  << "ms\n";

        telem_csv.close();

        // ── 9. Plot — profile CSV first, telem CSV second ─────────────────
        std::string plot_cmd = std::string("python3 ") + SCRIPT_DIR + plot_script + " " +
                               DOCS_DIR + prof_csv + " " +
                               DOCS_DIR + telem_f;

        int ret = system(plot_cmd.c_str());
        std::cout << "Plot exit code: " << ret << "\n";
    }

    spi_send_stop();
    spi_close();
    return 0;
}