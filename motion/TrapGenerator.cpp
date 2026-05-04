#include "TrapGenerator.hpp"
#include <chrono>
#include <cmath>

namespace Trajectory
{

TrapGenerator::TrapGenerator() {}
TrapGenerator::~TrapGenerator() {}

void TrapGenerator::Start(double startPos, double targetPos,
                          double vel, double accel)
{
    m_startPos   = startPos;
    m_target     = targetPos;
    m_vel        = vel;
    m_accel      = accel;
    m_posCmd     = startPos;
    m_direction  = (targetPos >= startPos) ? 1.0 : -1.0;

    double dist = m_direction * (targetPos - startPos);

    m_tAccel = vel / accel;
    m_dRamp = 0.5 * accel * m_tAccel * m_tAccel;

    m_triangular = (m_dRamp >= dist);
    if (m_triangular)
    {
        m_tAccel = std::sqrt(dist / accel);
        m_vel    = accel * m_tAccel;
        m_dRamp  = dist;
    }

    // Cruise duration
    double cruiseDist = m_direction * (m_target - m_startPos) - m_dRamp;
    m_tCruise = (m_triangular) ? 0.0 : cruiseDist / m_vel;

    // Total move time
    m_tTotal = 2.0 * m_tAccel + m_tCruise;

    m_cruiseStartPos = m_startPos + m_direction * 0.5 * m_accel * m_tAccel * m_tAccel;
    m_decelStartPos  = m_cruiseStartPos + m_direction * m_vel * m_tCruise;

   m_start = true;
   Run();
}

void TrapGenerator::Run()
{
    if (m_start)
    {
        m_startTime = std::chrono::steady_clock::now();
        m_start     = false;
        m_isDone    = false;
    }


    double t = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - m_startTime).count();

    if (t >= m_tTotal)
    {
        m_posCmd = m_target;
        m_isDone = true;
   
    }
    else if (t < m_tAccel)
    {
   
        m_posCmd = m_startPos + m_direction * 0.5 * m_accel * t * t;
    }
    else if (t < m_tAccel + m_tCruise)
    {
     
        double tC    = t - m_tAccel;
        m_posCmd     = m_cruiseStartPos + m_direction * m_vel * tC;
    }
    else
    {
       
        double tD    = t - m_tAccel - m_tCruise;
        m_posCmd     = m_decelStartPos + m_direction * (m_vel * tD - 0.5 * m_accel * tD * tD);
    }
}

double TrapGenerator::GetPosCmd() const
{
    return m_posCmd;
}

bool TrapGenerator::IsDone() const
{
    return m_isDone;
}


} // namespace Trajectory