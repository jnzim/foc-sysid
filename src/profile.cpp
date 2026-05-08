#include "profile.hpp"
#include <cmath>
#include <iostream>

std::vector<Sample> compute_profile(int32_t start_cnt,
                                    int32_t target_cnt,
                                    int32_t vel_cnt,
                                    int32_t accel_cnt,
                                    double  dt)
{
    // Work in double for math — int32_t at input/output boundaries only
    double start  = static_cast<double>(start_cnt);
    double target = static_cast<double>(target_cnt);
    double vel    = static_cast<double>(vel_cnt);
    double accel  = static_cast<double>(accel_cnt);

    // ── Trapezoid geometry ────────────────────────────────────────────────
    double direction = (target >= start) ? 1.0 : -1.0;
    double dist      = direction * (target - start);

    double tAccel = vel / accel;
    double dRamp  = 0.5 * accel * tAccel * tAccel;

    // Triangular profile if not enough distance to reach full velocity
    bool triangular = (dRamp >= dist / 2.0);
    if (triangular)
    {
        tAccel = std::sqrt(dist / accel);
        vel    = accel * tAccel;
        dRamp  = dist / 2.0;
    }

    double cruiseDist     = dist - 2.0 * dRamp;
    double tCruise        = triangular ? 0.0 : cruiseDist / vel;
    double tTotal         = 2.0 * tAccel + tCruise;
    double cruiseStartPos = start + direction * dRamp;
    double decelStartPos  = cruiseStartPos + direction * vel * tCruise;


    std::cout << "dist      : " << dist      << "\n";
    std::cout << "tAccel    : " << tAccel    << "\n";
    std::cout << "dRamp     : " << dRamp     << "\n";
    std::cout << "triangular: " << triangular << "\n";
    std::cout << "tCruise   : " << tCruise   << "\n";
    std::cout << "tTotal    : " << tTotal    << "\n";


    // ── Sample profile at fixed dt ────────────────────────────────────────
    // Use integer step counter — avoids floating point accumulation error
    int total_steps = static_cast<int>(std::round(tTotal / dt));

    std::vector<Sample> profile;
    profile.reserve(total_steps + 2);

    for (int step = 0; step <= total_steps; step++)
    {
        double t = step * dt;   // multiply not accumulate — no drift

        double pos, v;

        if (t < tAccel)
        {
            // Accel phase — velocity ramps up linearly
            pos = start + direction * 0.5 * accel * t * t;
            v   = direction * accel * t;
        }
        else if (t < tAccel + tCruise)
        {
            // Cruise phase — constant velocity
            double tC = t - tAccel;
            pos = cruiseStartPos + direction * vel * tC;
            v   = direction * vel;
        }
        else
        {
            // Decel phase — velocity ramps down linearly
            double tD = t - tAccel - tCruise;
            pos = decelStartPos + direction * (vel * tD - 0.5 * accel * tD * tD);
            v   = direction * (vel - accel * tD);
        }

        Sample s;
        s.pos = static_cast<int32_t>(std::round(pos));
        s.vel = static_cast<int32_t>(std::round(v));
        profile.push_back(s);
    }

    // Guarantee final sample is exactly at target with zero velocity
    Sample last;
    last.pos = target_cnt;
    last.vel = 0;
    profile.push_back(last);

    return profile;
}