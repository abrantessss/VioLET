#pragma once

#include <algorithm>
#include <cmath>

namespace autopilot {
// Armed, steady-state Gazebo mapping: omega = min(scale*c + idle, omega_max).
// The current models use omega_max as their input scaling as well.
inline double thrust_to_motor_command(double thrust, double k, double omega_max, double idle)
{
  const double target = std::isfinite(thrust) ? std::max(thrust, 0.0) : 0.0;
  const double omega = std::clamp(std::sqrt(target / k), idle, omega_max);
  return std::clamp((omega - idle) / omega_max, 0.0, 1.0);
}

inline double motor_command_to_thrust(double command, double k, double omega_max, double idle)
{
  const double c = std::isfinite(command) ? std::clamp(command, 0.0, 1.0) : 0.0;
  const double omega = std::min(omega_max * c + idle, omega_max);
  return k * omega * omega;
}
}
