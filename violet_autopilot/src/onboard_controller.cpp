#include "onboard_controller.hpp"

namespace autopilot {
  
  OnboardController::~OnboardController() {}

  void OnboardController::initialize() {
    node_->declare_parameter<std::string>("controllers.onboardcontroller.publishers.setpoint", "fmu/in/trajectory_setpoint");
    position_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(node_->get_parameter("controllers.onboardcontroller.publishers.setpoint").as_string(), rclcpp::SensorDataQoS());
    
    node_->declare_parameter<std::string>("publishers.mode.offboard", "fmu/in/offboard_control_mode");
    offboard_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(node_->get_parameter("publishers.mode.offboard").as_string(), rclcpp::SensorDataQoS());
    
    node_->declare_parameter<std::string>("publishers.mode.request", "fmu/in/vehicle_command");
    mode_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(node_->get_parameter("publishers.mode.request").as_string(), rclcpp::SensorDataQoS());

    // Log that the OnboardController was initialized
    RCLCPP_INFO(node_->get_logger(), "OnboardController initialized");
  }

  void OnboardController::set_position(const double dt, const Eigen::Vector3d& p) {
    Eigen::Vector3d pd = Eigen::Vector3d::Zero();
    Eigen::Vector3d ep = Eigen::Vector3d::Zero();
    double gamma_dot = 0.0;

    if (path_.type == 0) {
      // Waypoint
      pd = path_.waypoint;
      ep = p - pd;
      gamma_dot = 0.0;
    }
    else if (path_.type == 1) {
      // Line
      const double vd = path_.line_v;

      pd = path_.line_p0 + gamma_ * (path_.line_p1 - path_.line_p0);
      ep = p - pd;

      const double ep_norm = ep.norm();

      if (ep_norm >= 1.5) {
        gamma_dot = 0.0;
      } else if (ep_norm >= 0.5) {
        gamma_dot = vd * std::exp(1.0 + 1.0 / (((ep_norm - 0.5) * (ep_norm - 0.5)) - 1.0));
      } else {
        gamma_dot = vd;
      }

      gamma_ += gamma_dot * dt;
      gamma_ = std::clamp(gamma_, 0.0, 1.0);
    }
    else if (path_.type == 2) {
      // Circle
      const double vd = path_.circle_v;
      const double R = path_.circle_R;
      const Eigen::Vector3d& c = path_.circle_c;

      pd << c.x() + R * std::cos(gamma_),
            c.y() + R * std::sin(gamma_),
            c.z();

      ep = p - pd;

      const double ep_norm = ep.norm();

      if (ep_norm >= 1.5) {
        gamma_dot = 0.0;
      } else if (ep_norm >= 0.5) {
        gamma_dot = vd * std::exp(1.0 + 1.0 / (((ep_norm - 0.5) * (ep_norm - 0.5)) - 1.0));
      } else {
        gamma_dot = vd;
      }

      gamma_ += gamma_dot * dt;
    }
    else {
      // Lemniscate
      const double vd = path_.lemniscate_v;
      const double a = path_.lemniscate_a;
      const Eigen::Vector3d& c = path_.lemniscate_c;

      const double s = std::sin(gamma_);
      const double cg = std::cos(gamma_);
      const double denom = 1.0 + s * s;

      pd << c.x() + a * cg / denom,
            c.y() + a * s * cg / denom,
            c.z();

      ep = p - pd;

      const double ep_norm = ep.norm();

      if (ep_norm >= 1.5) {
        gamma_dot = 0.0;
      } else if (ep_norm >= 0.5) {
        gamma_dot = vd * std::exp(1.0 + 1.0 / (((ep_norm - 0.5) * (ep_norm - 0.5)) - 1.0));
      } else {
        gamma_dot = vd;
      }

      gamma_ += gamma_dot * dt;
    }

    const uint64_t now_us = node_->get_clock()->now().nanoseconds() / 1000;

    position_msg_.timestamp = now_us;
    position_msg_.position[0] = static_cast<float>(pd.x());
    position_msg_.position[1] = static_cast<float>(pd.y());
    position_msg_.position[2] = static_cast<float>(pd.z());
    position_msg_.yaw = 0.0f;

    position_pub_->publish(position_msg_);

    offboard_msg_.timestamp = now_us;
    offboard_msg_.position = true;
    offboard_msg_.velocity = false;
    offboard_msg_.acceleration = false;
    offboard_msg_.attitude = false;
    offboard_msg_.body_rate = false;
    offboard_msg_.thrust_and_torque = false;
    offboard_msg_.direct_actuator = false;

    offboard_pub_->publish(offboard_msg_);
  }

  void OnboardController::set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) {
    (void)dt;
    (void)p;
    (void)v;
    (void)eta;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "OnboardController does not support attitude-rate control");
  }

  void OnboardController::set_path(const int type, const double* path) {
    path_.type = type;
    gamma_ = 0.0;

    if (path_.type == 0) {
      path_.waypoint << path[0], path[1], path[2];
    }
    else if (path_.type == 1) {
      path_.line_p0 << path[0], path[1], path[2];
      path_.line_p1 << path[3], path[4], path[5];
      path_.line_v = path[6];
    }
    else if (path_.type == 2) {
      path_.circle_c << path[0], path[1], path[2];
      path_.circle_R = path[3];
      path_.circle_v = path[4];
    }
    else {
      path_.lemniscate_c << path[0], path[1], path[2];
      path_.lemniscate_a = path[3];
      path_.lemniscate_v = path[4];
    }

    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    msg.param1 = 1;
    msg.param2 = 6;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    mode_pub_->publish(msg);
  }
}
