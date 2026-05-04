#pragma once
#include <chrono>

namespace Trajectory
{

enum class TrapState
{
    IDLE,
    ACCEL,
    CRUISE,
    DECEL,
    DONE
};

class TrapGenerator
{
public:
    TrapGenerator();
    ~TrapGenerator();

    void      Start(double startPos, double targetPos, double vel, double accel);
    void      Run();
    double    GetPosCmd() const;
    bool      IsDone()   const;
    TrapState GetState() const;

private:
    // Config
    double m_startPos    {0.0};
    double m_target      {0.0};
    double m_vel         {0.0};
    double m_accel       {0.0};
    double m_direction   {1.0};

    // Computed profile params
    double m_tAccel      {0.0};
    double m_tCruise     {0.0};
    double m_tTotal      {0.0};
    double m_dRamp       {0.0};
    bool   m_triangular  {false};

    // Precomputed phase entry positions
    double m_cruiseStartPos {0.0};
    double m_decelStartPos  {0.0};

    // Runtime
    double m_posCmd {0.0};
    std::chrono::steady_clock::time_point m_startTime;

    TrapState m_state {TrapState::IDLE};
    bool      m_start {false};
};

} // namespace Trajectory