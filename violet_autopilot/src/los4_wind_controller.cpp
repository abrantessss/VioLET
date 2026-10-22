#include "los4_wind_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace autopilot {
void LOS4WindController::initialize()
{
  LOS4Controller::initialize();
  const auto topic = node_->declare_parameter<std::string>(
    "controllers.los4windcontroller.airspeed_topic", "fmu/out/airspeed_validated_v1");
  airspeed_timeout_ = node_->declare_parameter<double>(
    "controllers.los4windcontroller.airspeed_timeout", 1.0);
  if (!std::isfinite(airspeed_timeout_) || airspeed_timeout_ <= 0.0) {
    throw std::invalid_argument("los4_wind airspeed_timeout must be positive and finite");
  }
  const auto wind_topic = node_->declare_parameter<std::string>(
    "controllers.los4windcontroller.wind_topic", "fmu/out/wind");
  wind_timeout_ = node_->declare_parameter<double>(
    "controllers.los4windcontroller.wind_timeout", 2.0);
  if (!std::isfinite(wind_timeout_) || wind_timeout_ <= 0.0) {
    throw std::invalid_argument("los4_wind wind_timeout must be positive and finite");
  }
  have_wind_ = false;
  wind_sub_ = node_->create_subscription<px4_msgs::msg::Wind>(
    wind_topic, rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::Wind::ConstSharedPtr msg) {
      // Zero variance denotes an unestimated wind state in PX4's Wind message.
      have_wind_ = std::isfinite(msg->windspeed_north) && std::isfinite(msg->windspeed_east) &&
        std::isfinite(msg->variance_north) && msg->variance_north > 0.0f &&
        std::isfinite(msg->variance_east) && msg->variance_east > 0.0f;
      if (have_wind_) {
        wind_ne_ << msg->windspeed_north, msg->windspeed_east;
        wind_received_ = node_->now();
      }
    });
  have_airspeed_ = false;
  airspeed_sub_ = node_->create_subscription<px4_msgs::msg::AirspeedValidated>(
    topic, rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::AirspeedValidated::ConstSharedPtr msg) {
      const double speed = msg->true_airspeed_m_s;
      have_airspeed_ = msg->airspeed_source >= 0 && msg->airspeed_source <= 4 &&
        std::isfinite(speed) && speed >= 0.0;
      if (have_airspeed_) {
        true_airspeed_ = speed;
        airspeed_received_ = node_->now();
      }
    });
}

bool LOS4WindController::energy_speed(const Eigen::Vector3d& velocity, double& speed)
{
  (void)velocity;
  const double age = have_airspeed_ ? (node_->now() - airspeed_received_).seconds() : -1.0;
  if (!have_airspeed_ || age < 0.0 || age > airspeed_timeout_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "LOS4WindController waiting for fresh valid true airspeed; rate/thrust setpoints paused");
    return false;
  }
  speed = true_airspeed_;
  return true;
}

// NPFG-style on-path wind-triangle conversion, for steady horizontal wind.
// Returns NaN when the requested forward path bearing is infeasible.
double LOS4WindController::wind_curvature_acceleration(
  double curvature, const Eigen::Vector2d& tangent,
  const Eigen::Vector2d& ground_velocity, double airspeed, const Eigen::Vector2d& wind)
{
  const double along_wind = wind.dot(tangent);
  const double cross_wind = tangent.x() * wind.y() - tangent.y() * wind.x();
  const double projected_airspeed_sq = airspeed * airspeed - cross_wind * cross_wind;
  if (!std::isfinite(airspeed) || airspeed <= 0.0 || projected_airspeed_sq <= 1e-6) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double projected_airspeed = std::sqrt(projected_airspeed_sq);
  const double predicted_ground_speed = projected_airspeed + along_wind;
  if (predicted_ground_speed <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double speed_ratio = predicted_ground_speed / projected_airspeed;
  const double tangent_ground_speed = std::max(ground_velocity.dot(tangent), 0.0);
  return airspeed * speed_ratio * curvature * tangent_ground_speed;
}

double LOS4WindController::curvature_feedforward(
  double curvature, const Eigen::Vector2d& tangent,
  const Eigen::Vector3d& ground_velocity, double airspeed)
{
  const double age = have_wind_ ? (node_->now() - wind_received_).seconds() : -1.0;
  if (!have_wind_ || age < 0.0 || age > wind_timeout_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "LOS4 wind estimate unavailable or stale; using baseline curvature feedforward");
    return LOS4Controller::curvature_feedforward(curvature, tangent, ground_velocity, airspeed);
  }
  const double acceleration = wind_curvature_acceleration(
    curvature, tangent, ground_velocity.head<2>(), airspeed, wind_ne_);
  if (!std::isfinite(acceleration)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "LOS4 wind: path bearing infeasible at reference airspeed; curvature feedforward disabled");
    return 0.0;  // Retain bounded direction feedback; no excess-wind recovery is implemented.
  }
  return acceleration;
}

}
