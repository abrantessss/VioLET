#include <gtest/gtest.h>
#include "los4_energy_control.hpp"

TEST(LOS4Energy, SpeedAndAltitudeCorrectionsHavePhysicalSigns)
{
  const double slow = autopilot::kinetic_energy_error(1.48, 18, 20);
  const double fast = autopilot::kinetic_energy_error(1.48, 22, 20);
  EXPECT_LT(slow, 0);
  EXPECT_GT(fast, 0);
  auto command = [](double error) {
    double integral = 0;
    return autopilot::throttle_command_with_antiwindup(error, .1, integral, .005, 100, .05, 20, 100, 0, 1);
  };
  EXPECT_GT(command(slow), command(0));
  EXPECT_LT(command(fast), command(0));
  EXPECT_GT(command(-20), command(0));  // Below reference: energy deficit.
  EXPECT_LT(command(20), command(0));   // Above reference: energy surplus.
  // Existing pitch law B = K_err-U_err: slow => nose down; low => nose up.
  EXPECT_LT(slow - 0.0, 0);
  EXPECT_GT(0.0 - (-20.0), 0);
}

TEST(LOS4Energy, IntegralAgreesWithProportionalFeedback)
{
  double surplus_integral = 0, deficit_integral = 0;
  const double base = std::sqrt(.05);
  double high = base, low = base;
  for (int i = 0; i < 10; ++i) {
    high = autopilot::throttle_command_with_antiwindup(10, .1, surplus_integral, .01, 100, 0, 20, 100, 0, 1);
    low = autopilot::throttle_command_with_antiwindup(-10, .1, deficit_integral, .01, 100, 0, 20, 100, 0, 1);
  }
  EXPECT_LT(high, base);
  EXPECT_GT(low, base);
}

TEST(LOS4Energy, EnforcesThrottleLimitsAndUnwindsInCorrectDirection)
{
  double integral = 0;
  EXPECT_DOUBLE_EQ(autopilot::throttle_command_with_antiwindup(-100, 1, integral, 1, 10000, 1, 20, 100, .2, .6), .6);
  EXPECT_DOUBLE_EQ(integral, 0);  // Deficit would wind up further at upper limit.
  autopilot::throttle_command_with_antiwindup(1, 1, integral, 1, 10000, 1, 20, 100, .2, .6);
  EXPECT_DOUBLE_EQ(integral, 1);  // Surplus unwinds upper saturation.
  integral = 0;
  EXPECT_NEAR(autopilot::throttle_command_with_antiwindup(100, 1, integral, 1, 0, 1, 20, 100, .2, .6), .2, 1e-12);
  EXPECT_DOUBLE_EQ(integral, 0);
  autopilot::throttle_command_with_antiwindup(-1, 1, integral, 1, -1000, 1, 20, 100, .2, .6);
  EXPECT_DOUBLE_EQ(integral, -1); // Deficit unwinds lower saturation.
}
