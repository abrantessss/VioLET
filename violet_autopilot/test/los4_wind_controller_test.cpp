#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include "los4_wind_controller.hpp"

class WindHarness : public autopilot::LOS4WindController {
public:
  using autopilot::LOS4WindController::energy_speed;
  using autopilot::LOS4WindController::curvature_feedforward;
  using autopilot::LOS4WindController::wind_curvature_acceleration;
};
class BaselineHarness : public autopilot::LOS4Controller {
public:
  using autopilot::LOS4Controller::energy_speed;
};

TEST(LOS4Wind, UsesValidatedAirspeedAndRejectsMissingInvalidAndStaleData)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("controllers.los4windcontroller.airspeed_timeout", 0.1)});
  {
    auto node = std::make_shared<rclcpp::Node>("controller", "/los4_wind_test", options);
    WindHarness controller;
    controller.initialize_controller({node, 1});
    double speed = 0.0;
    const Eigen::Vector3d ground_velocity(30, 0, 0);
    EXPECT_FALSE(controller.energy_speed(ground_velocity, speed));
    BaselineHarness baseline;
    EXPECT_TRUE(baseline.energy_speed(ground_velocity, speed));
    EXPECT_DOUBLE_EQ(speed, 30.0);
    auto pub = node->create_publisher<px4_msgs::msg::AirspeedValidated>(
      "fmu/out/airspeed_validated_v1", rclcpp::SensorDataQoS());
    auto deliver = [&](float tas, int source) {
      px4_msgs::msg::AirspeedValidated msg;
      msg.true_airspeed_m_s = tas;
      msg.airspeed_source = source;
      const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
      do {
        pub->publish(msg);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      } while (std::chrono::steady_clock::now() < end);
    };
    deliver(14.0f, 1);
    EXPECT_TRUE(controller.energy_speed(ground_velocity, speed));
    EXPECT_DOUBLE_EQ(speed, 14.0);
    deliver(std::numeric_limits<float>::quiet_NaN(), 1);
    EXPECT_FALSE(controller.energy_speed(ground_velocity, speed));
    deliver(14.0f, -1);
    EXPECT_FALSE(controller.energy_speed(ground_velocity, speed));
    deliver(12.0f, 0);
    EXPECT_TRUE(controller.energy_speed(ground_velocity, speed));
    EXPECT_DOUBLE_EQ(speed, 12.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    EXPECT_FALSE(controller.energy_speed(ground_velocity, speed));
  }
  rclcpp::shutdown();
}

TEST(LOS4Wind, CurvatureFeedforwardWindTriangle)
{
  const Eigen::Vector2d north(1, 0);
  const double k = 0.01;
  EXPECT_NEAR(WindHarness::wind_curvature_acceleration(k, north, {20, 0}, 20, {0, 0}), 4.0, 1e-12);
  EXPECT_NEAR(WindHarness::wind_curvature_acceleration(k, north, {25, 0}, 20, {5, 0}), 6.25, 1e-12);
  EXPECT_NEAR(WindHarness::wind_curvature_acceleration(k, north, {15, 0}, 20, {-5, 0}), 2.25, 1e-12);
  const double vg = std::sqrt(400.0 - 25.0);
  EXPECT_NEAR(WindHarness::wind_curvature_acceleration(k, north, {vg, 0}, 20, {0, 5}),
    k * vg * vg / (vg / 20.0), 1e-12);
  EXPECT_NEAR(WindHarness::wind_curvature_acceleration(-k, north, {25, 0}, 20, {5, 0}), -6.25, 1e-12);
  EXPECT_DOUBLE_EQ(WindHarness::wind_curvature_acceleration(0, north, {25, 0}, 20, {5, 0}), 0.0);
  EXPECT_TRUE(std::isnan(WindHarness::wind_curvature_acceleration(k, north, {0, 0}, 20, {0, 21})));
  EXPECT_TRUE(std::isnan(WindHarness::wind_curvature_acceleration(k, north, {0, 0}, 20, {-21, 0})));
}

TEST(LOS4Wind, UsesWindTopicAndFallsBackWhenUnavailableInvalidOrStale)
{
  rclcpp::init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("controllers.los4windcontroller.wind_timeout", 0.1)});
    auto node = std::make_shared<rclcpp::Node>("wind_controller", "/los4_wind_test", options);
    WindHarness controller;
    controller.initialize_controller({node, 1});
    auto accel = [&]() { return controller.curvature_feedforward(0.01, {1, 0}, {15, 0, 0}, 20); };
    EXPECT_DOUBLE_EQ(accel(), 4.0);  // No estimate: baseline, not the old hardcoded south wind.
    auto pub = node->create_publisher<px4_msgs::msg::Wind>("fmu/out/wind", rclcpp::SensorDataQoS());
    auto deliver = [&](float north, float variance) {
      px4_msgs::msg::Wind msg;
      msg.windspeed_north = north;
      msg.windspeed_east = 0;
      msg.variance_north = variance;
      msg.variance_east = variance;
      const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
      do {
        pub->publish(msg);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      } while (std::chrono::steady_clock::now() < end);
    };
    deliver(-5, 0.2);
    EXPECT_NEAR(accel(), 2.25, 1e-12);
    deliver(5, 0.2);
    EXPECT_NEAR(accel(), 3.75, 1e-12);  // Updated estimate is used, not fixed world wind.
    deliver(0, 0.2);
    EXPECT_NEAR(accel(), 3.0, 1e-12);  // A valid zero-wind estimate is accepted.
    deliver(-5, 0);
    EXPECT_DOUBLE_EQ(accel(), 4.0);
    deliver(std::numeric_limits<float>::quiet_NaN(), 0.2);
    EXPECT_DOUBLE_EQ(accel(), 4.0);
    deliver(-5, 0.2);
    EXPECT_NEAR(accel(), 2.25, 1e-12);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    EXPECT_DOUBLE_EQ(accel(), 4.0);
  }
  rclcpp::shutdown();
}
