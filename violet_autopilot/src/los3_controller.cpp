#include "los3_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace autopilot {
  namespace {
    constexpr double PI = 3.14159265358979323846;

    double pi_command_with_antiwindup(
      const double error,
      const double dt,
      double& error_integral,
      const double kp,
      const double ki,
      const double command_min,
      const double command_max)
    {
      const double integral_candidate = error_integral + error * dt;
      const double command_unsaturated = kp * error + ki * integral_candidate;
      const double command = std::clamp(command_unsaturated, command_min, command_max);

      const bool saturated_high = command_unsaturated > command_max;
      const bool saturated_low = command_unsaturated < command_min;
      const bool drives_out_of_high_saturation = saturated_high && error < 0.0;
      const bool drives_out_of_low_saturation = saturated_low && error > 0.0;

      if ((!saturated_high && !saturated_low) ||
          drives_out_of_high_saturation ||
          drives_out_of_low_saturation) {
        error_integral = integral_candidate;
      }

      return command;
    }

    Eigen::Vector3d lemniscate_derivative(const double a, const double gamma)
    {
      Eigen::Vector3d derivative;
      derivative << a * std::sin(gamma) * (std::pow(std::sin(gamma), 2) - 3) /
                      std::pow(1 + std::pow(std::sin(gamma), 2), 2),
                    a * (1 - 3 * std::pow(std::sin(gamma), 2)) /
                      std::pow(1 + std::pow(std::sin(gamma), 2), 2),
                    0.0;
      return derivative;
    }

    double signed_horizontal_curvature(
      const Eigen::Vector3d& dpd_dgamma,
      const Eigen::Vector3d& d2pd_dgamma2)
    {
      const double horizontal_speed_sq =
        dpd_dgamma.x() * dpd_dgamma.x() + dpd_dgamma.y() * dpd_dgamma.y();
      const double denominator = std::pow(horizontal_speed_sq, 1.5);

      return (dpd_dgamma.x() * d2pd_dgamma2.y() -
              dpd_dgamma.y() * d2pd_dgamma2.x()) / denominator;
    }
  }
  
  LOS3Controller::~LOS3Controller() {}

  void LOS3Controller::initialize() {
    node_->declare_parameter<std::string>("controllers.los3controller.publishers.longitudinal", "fmu/in/fixed_wing_longitudinal_setpoint");
    lon_pub_ = node_->create_publisher<px4_msgs::msg::FixedWingLongitudinalSetpoint>(node_->get_parameter("controllers.los3controller.publishers.longitudinal").as_string(), rclcpp::SensorDataQoS());

    node_->declare_parameter<std::string>("controllers.los3controller.publishers.lateral", "fmu/in/fixed_wing_lateral_setpoint");
    lat_pub_ = node_->create_publisher<px4_msgs::msg::FixedWingLateralSetpoint>(node_->get_parameter("controllers.los3controller.publishers.lateral").as_string(), rclcpp::SensorDataQoS());
    
    node_->declare_parameter<std::string>("publishers.mode.offboard", "fmu/in/offboard_control_mode");
    offboard_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(node_->get_parameter("publishers.mode.offboard").as_string(), rclcpp::SensorDataQoS());
    
    node_->declare_parameter<std::string>("publishers.mode.request", "fmu/in/vehicle_command");
    mode_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(node_->get_parameter("publishers.mode.request").as_string(), rclcpp::SensorDataQoS());

    // Load Mass
    node_->declare_parameter<double>("controllers.los3controller.m", 1.5);

    // Load Gains
    node_->declare_parameter<double>("controllers.los3controller.gains.k1", 1.0);
    node_->declare_parameter<double>("controllers.los3controller.gains.k2", 1.0);
    node_->declare_parameter<double>("controllers.los3controller.gains.kpE", 1.0);
    node_->declare_parameter<double>("controllers.los3controller.gains.kiE", 0.0);
    node_->declare_parameter<double>("controllers.los3controller.gains.kpB", 1.0);
    node_->declare_parameter<double>("controllers.los3controller.gains.kiB", 0.0);
    node_->declare_parameter<double>("controllers.los3controller.gains.ka", 1.0);
    node_->declare_parameter<double>("controllers.los3controller.limits.throttle_min", 0.0);
    node_->declare_parameter<double>("controllers.los3controller.limits.throttle_max", 1.0);
    node_->declare_parameter<double>("controllers.los3controller.limits.pitch_min", -PI);
    node_->declare_parameter<double>("controllers.los3controller.limits.pitch_max", PI);
    node_->declare_parameter<double>("controllers.los3controller.limits.lateral_acceleration_min", -4.0);
    node_->declare_parameter<double>("controllers.los3controller.limits.lateral_acceleration_max", 4.0);

    m_ = node_->get_parameter("controllers.los3controller.m").as_double();
    k1_ = node_->get_parameter("controllers.los3controller.gains.k1").as_double();
    k2_ = node_->get_parameter("controllers.los3controller.gains.k2").as_double();
    kpE_ = node_->get_parameter("controllers.los3controller.gains.kpE").as_double();
    kiE_ = node_->get_parameter("controllers.los3controller.gains.kiE").as_double();
    kpB_ = node_->get_parameter("controllers.los3controller.gains.kpB").as_double();
    kiB_ = node_->get_parameter("controllers.los3controller.gains.kiB").as_double();
    ka_ = node_->get_parameter("controllers.los3controller.gains.ka").as_double();
    throttle_min_ = node_->get_parameter("controllers.los3controller.limits.throttle_min").as_double();
    throttle_max_ = node_->get_parameter("controllers.los3controller.limits.throttle_max").as_double();
    pitch_min_ = node_->get_parameter("controllers.los3controller.limits.pitch_min").as_double();
    pitch_max_ = node_->get_parameter("controllers.los3controller.limits.pitch_max").as_double();
    lateral_acceleration_min_ = node_->get_parameter("controllers.los3controller.limits.lateral_acceleration_min").as_double();
    lateral_acceleration_max_ = node_->get_parameter("controllers.los3controller.limits.lateral_acceleration_max").as_double();

    if (throttle_min_ > throttle_max_) {
      std::swap(throttle_min_, throttle_max_);
    }
    if (pitch_min_ > pitch_max_) {
      std::swap(pitch_min_, pitch_max_);
    }
    if (lateral_acceleration_min_ > lateral_acceleration_max_) {
      std::swap(lateral_acceleration_min_, lateral_acceleration_max_);
    }

    // Log Gains 
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS3Controller vehicle gain: k1 = " << k1_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS3Controller vehicle gain: k2 = " << k2_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS3Controller vehicle gain: kpE = " << kpE_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS3Controller vehicle gain: kiE = " << kiE_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS3Controller vehicle gain: kpB = " << kpB_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS3Controller vehicle gain: kiB = " << kiB_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS3Controller vehicle gain: ka = " << ka_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS3Controller throttle limits: [" << throttle_min_ << ", " << throttle_max_ << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS3Controller pitch limits: [" << pitch_min_ << ", " << pitch_max_ << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS3Controller lateral acceleration limits: [" << lateral_acceleration_min_ << ", " << lateral_acceleration_max_ << "]");

    // Log that the LOS3Controller was initialized
    RCLCPP_INFO(node_->get_logger(), "LOS3Controller initialized");
  }

  void LOS3Controller::set_position(const double dt, const Eigen::Vector3d& p) {
    (void)dt;
    (void)p;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "LOS3Controller does not support position control");
  }

  void LOS3Controller::set_attitude(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v) {
    Eigen::Vector3d pd = Eigen::Vector3d::Zero();
    Eigen::Vector3d ep = Eigen::Vector3d::Zero();
    Eigen::Vector3d dpd_dgamma = Eigen::Vector3d::Zero();
    Eigen::Vector3d d2pd_dgamma2 = Eigen::Vector3d::Zero();
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
      d2pd_dgamma2 = Eigen::Vector3d::Zero();
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

      d2pd_dgamma2 << -R*std::cos(gamma_),
                      -R*std::sin(gamma_),
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

      dpd_dgamma = lemniscate_derivative(a, gamma_);

      constexpr double derivative_step = 1e-4;
      d2pd_dgamma2 =
        (lemniscate_derivative(a, gamma_ + derivative_step) -
         lemniscate_derivative(a, gamma_ - derivative_step)) /
        (2.0 * derivative_step);
    }

    ep = p - pd;

    Eigen::Vector3d q = dpd_dgamma;
    q = q/q.norm();

    Eigen::Matrix3d PIq = Eigen::Matrix3d::Identity() - q*q.transpose();

    Eigen::Vector3d aux = -k1_*PIq*ep + k2_*q;
    Eigen::Vector3d h = aux/aux.norm();

    gamma_dot = k1_ * Va * q.dot(ep) / (aux.norm() * dpd_dgamma.norm()) + Va * k2_ / (aux.norm() * dpd_dgamma.norm());

    gamma_ += gamma_dot * dt;

    publish_plot_data(gamma_dot, Va, pd, dpd_dgamma, {
      static_cast<float>(k1_),
      static_cast<float>(k2_),
      static_cast<float>(kpE_),
      static_cast<float>(kiE_),
      static_cast<float>(kpB_),
      static_cast<float>(kiB_),
      static_cast<float>(ka_)});

    const double h_d_dot = - Va * h(2);

    if (!z_ref_initialized_) {
      z_ref_ = -p.z();
      z_ref_initialized_ = true;
    }

    z_ref_ += h_d_dot * dt;

    // Longitudinal Controller
    constexpr double g = 9.81;
    const double K_err = 0.5 * m_ * (Va*Va - v.squaredNorm());
    const double U_err = m_ * g * (z_ref_ - (-p.z()));

    const double E_err = U_err + K_err;
    const double B_err = U_err - K_err;

    const double throttle_cmd = pi_command_with_antiwindup(
      E_err,
      dt,
      E_err_int_,
      kpE_,
      kiE_,
      throttle_min_,
      throttle_max_);

    const double pitch_feedforward = h_d_dot / Va;
    const double pitch_feedback_cmd = pi_command_with_antiwindup(
      B_err,
      dt,
      B_err_int_,
      kpB_,
      kiB_,
      pitch_min_,
      pitch_max_);
    const double pitch_cmd = std::clamp(
      pitch_feedforward + pitch_feedback_cmd,
      pitch_min_,
      pitch_max_);

    // Lateral Controller
    Eigen::Vector3d h_current(v.x(), v.y(), 0.0);
    Eigen::Vector3d h_ref(h.x(), h.y(), 0.0);

    const double horizontal_speed = h_current.norm();
    const double horizontal_ref_norm = h_ref.norm();

    h_current /= horizontal_speed;
    h_ref /= horizontal_ref_norm;
    const double e_z = h_current.cross(h_ref)(2);

    const double lateral_acceleration_feedforward =
      Va * Va * signed_horizontal_curvature(dpd_dgamma, d2pd_dgamma2);

    const double lateral_acceleration_cmd = std::clamp(
      lateral_acceleration_feedforward + ka_ * e_z,
      lateral_acceleration_min_,
      lateral_acceleration_max_);

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      500,
      "LOS3 commands: throttle=%.3f pitch=%.3f lateral_acceleration=%.3f lateral_ff=%.3f, h_ref=%.3f",
      throttle_cmd,
      pitch_cmd,
      lateral_acceleration_cmd,
      lateral_acceleration_feedforward,
      z_ref_);

    const uint64_t now_us = node_->get_clock()->now().nanoseconds() / 1000;

    const float nan = std::numeric_limits<float>::quiet_NaN();

    // LONGITUDINAL: finite pitch_direct and throttle_direct bypass TECS.
    lon_msg_.timestamp = now_us;
    lon_msg_.altitude = nan;
    lon_msg_.height_rate = nan;
    lon_msg_.equivalent_airspeed = nan;
    lon_msg_.pitch_direct = static_cast<float>(pitch_cmd);
    lon_msg_.throttle_direct = static_cast<float>(throttle_cmd);
    lon_pub_->publish(lon_msg_);

    // LATERAL: finite lateral_acceleration bypasses course/airspeed-direction guidance inputs.
    lat_msg_.timestamp = now_us;
    lat_msg_.course = nan;
    lat_msg_.airspeed_direction = nan;
    lat_msg_.lateral_acceleration = static_cast<float>(lateral_acceleration_cmd);
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

  void LOS3Controller::set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) {
    (void)dt;
    (void)p;
    (void)v;
    (void)eta;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "LOS3Controller does not support attitude-rate control");
  }

  void LOS3Controller::set_path(const int type, const double* path) {
    path_.type = type;
    gamma_ = 0.0;
    z_ref_initialized_ = false;
    E_err_int_ = 0.0;
    B_err_int_ = 0.0;

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
