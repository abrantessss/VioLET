#include "los2_controller.hpp"

namespace autopilot {
  
  LOS2Controller::~LOS2Controller() {}

  void LOS2Controller::initialize() {
    node_->declare_parameter<std::string>("controllers.los2controller.publishers.longitudinal", "fmu/in/fixed_wing_longitudinal_setpoint");
    lon_pub_ = node_->create_publisher<px4_msgs::msg::FixedWingLongitudinalSetpoint>(node_->get_parameter("controllers.los2controller.publishers.longitudinal").as_string(), rclcpp::SensorDataQoS());

    node_->declare_parameter<std::string>("controllers.los2controller.publishers.lateral", "fmu/in/fixed_wing_lateral_setpoint");
    lat_pub_ = node_->create_publisher<px4_msgs::msg::FixedWingLateralSetpoint>(node_->get_parameter("controllers.los2controller.publishers.lateral").as_string(), rclcpp::SensorDataQoS());
    
    node_->declare_parameter<std::string>("publishers.mode.offboard", "fmu/in/offboard_control_mode");
    offboard_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(node_->get_parameter("publishers.mode.offboard").as_string(), rclcpp::SensorDataQoS());
    
    node_->declare_parameter<std::string>("publishers.mode.request", "fmu/in/vehicle_command");
    mode_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(node_->get_parameter("publishers.mode.request").as_string(), rclcpp::SensorDataQoS());

    // Load Gains
    node_->declare_parameter<double>("controllers.los2controller.gains.k1", 1.0);
    node_->declare_parameter<double>("controllers.los2controller.gains.k2", 1.0);

    k1_ = node_->get_parameter("controllers.los2controller.gains.k1").as_double();
    k2_ = node_->get_parameter("controllers.los2controller.gains.k2").as_double();

    // Log Gains 
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS2Controller vehicle gain: k1 = " << k1_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS2Controller vehicle gain: k2 = " << k2_);

    // Log that the LOS2Controller was initialized
    RCLCPP_INFO(node_->get_logger(), "LOS2Controller initialized");
  }

  void LOS2Controller::set_position(const double dt, const Eigen::Vector3d& p) {
    Eigen::Vector3d pd = Eigen::Vector3d::Zero();
    Eigen::Vector3d ep = Eigen::Vector3d::Zero();
    Eigen::Vector3d dpd_dgamma = Eigen::Vector3d::Zero();
    double gamma_dot = 0.0;
    double Va = 0.0;

    if (path_.type == 0) {
      // Waypoint
      Va = 14; // m/s
      pd = path_.waypoint;
      publish_plot_data(gamma_dot, 0.0, pd, dpd_dgamma, {static_cast<float>(k1_), static_cast<float>(k2_)});
      return;
    }
    else if (path_.type == 1) {
      // Line
      Va = path_.line_v;

      pd = path_.line_p0 + gamma_ * (path_.line_p1 - path_.line_p0);

      dpd_dgamma = path_.line_p1 - path_.line_p0;
    }
    else if (path_.type == 2) {
      // Circle
      Va = path_.circle_v;
      const double R = path_.circle_R;
      const Eigen::Vector3d& c = path_.circle_c;

      pd << c.x() + R * std::cos(gamma_),
            c.y() + R * std::sin(gamma_),
            c.z();
      
      dpd_dgamma << -R*std::sin(gamma_),
                    R*std::cos(gamma_),
                    0;
    }
    else {
      // Lemniscate
      Va = path_.lemniscate_v;
      const double a = path_.lemniscate_a;
      const Eigen::Vector3d& c = path_.lemniscate_c;

      const double s  = std::sin(gamma_);
      const double cg = std::cos(gamma_);
      const double denom = 1.0 + s * s;

      pd << c.x() + a * cg / denom,
            c.y() + a * s * cg / denom,
            c.z();

      dpd_dgamma << (a * std::sin(gamma_) * (std::pow(std::sin(gamma_),2) - 3) / std::pow(1 + std::pow(std::sin(gamma_),2), 2)),
                    (a * (1 - 3 * std::pow(std::sin(gamma_),2)) / std::pow(1 + std::pow(std::sin(gamma_),2), 2)),
                    0;
    }

    ep = p - pd;

    Eigen::Vector3d q = dpd_dgamma;
    q = q/q.norm();

    Eigen::Matrix3d PIq = Eigen::Matrix3d::Identity() - q*q.transpose();

    Eigen::Vector3d aux = -k1_*PIq*ep + k2_*q;
    Eigen::Vector3d h = aux/aux.norm();

    gamma_dot = k1_ * Va * q.dot(ep) / (aux.norm() * dpd_dgamma.norm()) + Va * k2_ / (aux.norm() * dpd_dgamma.norm());

    gamma_ += gamma_dot * dt;

    publish_plot_data(gamma_dot, Va, pd, dpd_dgamma, {static_cast<float>(k1_), static_cast<float>(k2_)});

    double gamma_cmd = std::asin(-h(2));
    double chi_cmd = std::atan2(h(1), h(0));

    const uint64_t now_us = node_->get_clock()->now().nanoseconds() / 1000;

    const float nan = std::numeric_limits<float>::quiet_NaN();

    // LONGITUDINAL
    lon_msg_.timestamp = now_us;
    lon_msg_.altitude = nan;                         // or actual AMSL altitude target
    lon_msg_.height_rate = static_cast<float>(Va * std::sin(gamma_cmd));                      // or climb/descent rate
    lon_msg_.equivalent_airspeed = static_cast<float>(Va);
    lon_msg_.pitch_direct = nan;                     // unless also setting throttle_direct
    lon_msg_.throttle_direct = nan;                  // must be finite together with pitch_direct
    lon_pub_->publish(lon_msg_);

    // LATERAL
    lat_msg_.timestamp = now_us;
    lat_msg_.course = static_cast<float>(chi_cmd);   // valid
    lat_msg_.airspeed_direction = nan;              // optional, overrides course if finite
    lat_msg_.lateral_acceleration = nan;             // optional feedforward
    lat_pub_->publish(lat_msg_);

    
    offboard_msg_.timestamp = now_us;
    offboard_msg_.position = true;
    offboard_msg_.velocity = true;
    offboard_msg_.acceleration = true;
    offboard_msg_.attitude = true;
    offboard_msg_.body_rate = true;
    offboard_msg_.thrust_and_torque = true;
    offboard_msg_.direct_actuator = true;

    offboard_pub_->publish(offboard_msg_);

  }

  void LOS2Controller::set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) {
    (void)dt;
    (void)p;
    (void)v;
    (void)eta;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "LOS2Controller does not support attitude-rate control");
  }

  void LOS2Controller::set_path(const int type, const double* path) {
    path_.type = type;
    gamma_ = 0.0;

    if (path_.type == 0) {
      path_.waypoint << path[0], path[1], path[2];
      return;
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
    msg.target_system = vehicle_id_;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    mode_pub_->publish(msg);
  }
}
