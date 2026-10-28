#pragma once

#include <algorithm>
#include <cmath>

namespace autopilot {
// Actual minus desired kinetic energy. Potential energy uses m*g*(h-h_d).
inline double kinetic_energy_error(double mass, double speed, double desired_speed)
{
  return 0.5 * mass * (speed * speed - desired_speed * desired_speed);
}

inline double throttle_command_with_antiwindup(
  double energy_error, double dt, double& integral, double ki,
  double feedforward_power, double kp, double desired_speed, double tmax,
  double throttle_min, double throttle_max)
{
  const double minimum = std::clamp(throttle_min, 0.0, 1.0);
  const double maximum = std::clamp(throttle_max, minimum, 1.0);
  const double lower_thrust = minimum * minimum;
  const double upper_thrust = maximum * maximum;
  const double candidate = integral +
    ((dt > 0.0 && std::isfinite(dt)) ? energy_error * dt : 0.0);
  auto thrust_fraction = [&](double state) {
    return ((feedforward_power - kp * energy_error) / desired_speed - ki * state) / tmax;
  };
  const double unsaturated = thrust_fraction(candidate);
  // Positive energy surplus must reduce thrust through both P and I feedback.
  const bool inside = unsaturated >= lower_thrust && unsaturated <= upper_thrust;
  const bool reducing_high = unsaturated > upper_thrust && energy_error > 0.0;
  const bool reducing_low = unsaturated < lower_thrust && energy_error < 0.0;
  if (inside || reducing_high || reducing_low) {
    integral = candidate;
  }
  // Gazebo maps normalized throttle to speed, with T = Tmax * throttle^2.
  return std::sqrt(std::clamp(thrust_fraction(integral), lower_thrust, upper_thrust));
}
}
