#pragma once

#include "los4_controller.hpp"
#include "px4_msgs/msg/airspeed_validated.hpp"
#include "px4_msgs/msg/wind.hpp"

namespace autopilot {
// LOS4 with true-airspeed energy feedback and steady horizontal wind curvature compensation.
class LOS4WindController : public LOS4Controller {
public:
  void initialize() override;

protected:
  bool energy_speed(const Eigen::Vector3d& velocity, double& speed) override;
  double curvature_feedforward(double curvature, const Eigen::Vector2d& tangent,
    const Eigen::Vector3d& ground_velocity, double airspeed) override;
  static double wind_curvature_acceleration(double curvature, const Eigen::Vector2d& tangent,
    const Eigen::Vector2d& ground_velocity, double airspeed, const Eigen::Vector2d& wind);
  const char* results_name() const override { return "los4_wind"; }

private:
  rclcpp::Subscription<px4_msgs::msg::AirspeedValidated>::SharedPtr airspeed_sub_;
  rclcpp::Subscription<px4_msgs::msg::Wind>::SharedPtr wind_sub_;
  Eigen::Vector2d wind_ne_{Eigen::Vector2d::Zero()};
  bool have_wind_{false};
  double wind_timeout_{2.0};
  rclcpp::Time wind_received_{0, 0, RCL_ROS_TIME};
  double true_airspeed_{0.0};
  double airspeed_timeout_{1.0};
  bool have_airspeed_{false};
  rclcpp::Time airspeed_received_{0, 0, RCL_ROS_TIME};
};
}
