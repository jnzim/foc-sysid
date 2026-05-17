// chirp.cpp — linear sine sweep profile generator
// Sweeps from f_start to f_end Hz over duration seconds
// Output: 1kHz sampled position + velocity samples (encoder counts)

#include "chirp.hpp"
#include <cmath>

std::vector<Sample> compute_chirp(int32_t amplitude_counts,
                                   double f_start_hz,
                                   double f_end_hz,
                                   double duration_s)
{
    const double dt      = 0.001;                        // 1kHz
    const int    n       = (int)(duration_s / dt);
    const double k       = (f_end_hz - f_start_hz) / duration_s;  // Hz/s sweep rate

    std::vector<Sample> profile;
    profile.reserve(n);

    for (int i = 0; i < n; i++) {
        double t = i * dt;

        // Instantaneous frequency: f(t) = f_start + k*t
        // Phase: phi(t) = 2*pi * (f_start*t + 0.5*k*t^2)
        double phi = 2.0 * M_PI * (f_start_hz * t + 0.5 * k * t * t);

        // Position: A * sin(phi)
        double pos = amplitude_counts * std::sin(phi);

        // Velocity: d/dt [A*sin(phi)] = A * cos(phi) * dphi/dt
        // dphi/dt = 2*pi*(f_start + k*t)
        double dphi_dt = 2.0 * M_PI * (f_start_hz + k * t);
        double vel = amplitude_counts * std::cos(phi) * dphi_dt;  // counts/s

        Sample s;
        s.pos = (int32_t)pos;
        s.vel = (int32_t)vel;
        profile.push_back(s);
    }

    return profile;
}