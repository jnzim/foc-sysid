#include "profile.hpp"
#include "config.hpp"
#include <cmath>
#include <iostream>

std::vector<Sample> compute_profile(int32_t start_cnt,
                                    int32_t target_cnt,
                                    int32_t vel_cnt,
                                    int32_t accel_cnt,
                                    double  dt)
{
    double start  = static_cast<double>(start_cnt);
    double target = static_cast<double>(target_cnt);
    double vel    = static_cast<double>(vel_cnt);
    double accel  = static_cast<double>(accel_cnt);
    
    double direction = (target >= start) ? 1.0 : -1.0;
    double dist      = direction * (target - start);
    double tAccel    = vel / accel;
    double dRamp     = 0.5 * accel * tAccel * tAccel;
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
    
    int total_steps = static_cast<int>(std::round(tTotal / dt));
    
    // Debug output with both counts and mm
    std::cout << "=== Profile ===\n";
    std::cout << "dist       : " << dist << " cnts (" 
              << (dist / MACHINE.counts_per_mm()) << " mm)\n";
    std::cout << "vel        : " << vel << " cnts/s (" 
              << (vel / MACHINE.counts_per_mm()) << " mm/s)\n";
    std::cout << "accel      : " << accel << " cnts/s² (" 
              << (accel / MACHINE.counts_per_mm()) << " mm/s²)\n";
    std::cout << "decel      : " << accel << " cnts/s² (" 
              << (accel / MACHINE.counts_per_mm()) << " mm/s²)\n";
    std::cout << "\n=== Timing ===\n";
    std::cout << "t_accel    : " << tAccel << " s\n";
    std::cout << "t_cruise   : " << tCruise << " s\n";
    std::cout << "t_decel    : " << tAccel << " s\n";
    std::cout << "t_total    : " << tTotal << " s\n";
    std::cout << "triangular : " << (triangular ? "yes" : "no") << "\n";
    std::cout << "\n=== Samples ===\n";
    std::cout << "profile points : " << (total_steps + 2) << "\n";
    
    std::vector<Sample> profile;
    profile.reserve(total_steps + 2);
    
    // Filter state
    double vel_filt = 0.0;
    const double TAU = 0.050;
    const double alpha = dt / (TAU + dt);
    
    for (int step = 0; step <= total_steps; step++)
    {
        double t = step * dt;
        double pos, v;
        
        if (t < tAccel)
        {
            pos = start + direction * 0.5 * accel * t * t;
            v   = direction * accel * t;
        }
        else if (t < tAccel + tCruise)
        {
            double tC = t - tAccel;
            pos = cruiseStartPos + direction * vel * tC;
            v   = direction * vel;
        }
        else
        {
            double tD = t - tAccel - tCruise;
            pos = decelStartPos + direction * (vel * tD - 0.5 * accel * tD * tD);
            v   = direction * (vel - accel * tD);
        }
        
        vel_filt = vel_filt + alpha * (v - vel_filt);
        
        Sample s;
        s.pos = static_cast<int32_t>(std::round(pos));
        s.vel = static_cast<int32_t>(std::round(v));
        profile.push_back(s);
    }
    
    Sample last;
    last.pos = target_cnt;
    last.vel = 0;
    profile.push_back(last);
    
    return profile;
}