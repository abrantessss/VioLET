#include <gtest/gtest.h>
#include <limits>
#include "motor_thrust_model.hpp"

TEST(MotorThrustModel, QuadThrustRoundTripIncludesIdle)
{
  const double k = 1.709716e-5;
  for (double thrust : {0.5, 9.05, 12.0, 25.0, k * 1400 * 1400}) {
    const double c = autopilot::thrust_to_motor_command(thrust, k, 1400, 100);
    EXPECT_NEAR(c, (std::sqrt(thrust / k) - 100) / 1400, 1e-12);
    EXPECT_NEAR(autopilot::motor_command_to_thrust(c, k, 1400, 100), thrust, 1e-10);
  }
  // The old command for 9.05 N actually requested substantially more thrust.
  const double old_c = std::sqrt(9.05 / (k * 1400 * 1400));
  EXPECT_GT(autopilot::motor_command_to_thrust(old_c, k, 1400, 100), 11.7);
}

TEST(MotorThrustModel, ArmedIdleAndSpeedCapAreRespected)
{
  const double k = 1.709716e-5;
  for (double thrust : {0.0, -5.0, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_DOUBLE_EQ(autopilot::thrust_to_motor_command(thrust, k, 1400, 100), 0.0);
  }
  EXPECT_NEAR(autopilot::motor_command_to_thrust(0, k, 1400, 100), 0.1709716, 1e-12);
  EXPECT_NEAR(autopilot::thrust_to_motor_command(1000, k, 1400, 100), 1300.0 / 1400, 1e-12);
  EXPECT_NEAR(autopilot::motor_command_to_thrust(1, k, 1400, 100), k * 1400 * 1400, 1e-12);
}

TEST(MotorThrustModel, FixedWingZeroOffsetRemainsUnchanged)
{
  const double k = 8.54858e-6;
  for (double thrust : {0.0, 1.5, 50.0, k * 3500 * 3500}) {
    const double c = autopilot::thrust_to_motor_command(thrust, k, 3500, 0);
    EXPECT_NEAR(c, std::sqrt(thrust / (k * 3500 * 3500)), 1e-12);
    EXPECT_NEAR(autopilot::motor_command_to_thrust(c, k, 3500, 0), thrust, 1e-10);
  }
}
