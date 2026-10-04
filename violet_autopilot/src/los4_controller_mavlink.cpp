#include "los4_controller_mavlink.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <stdexcept>

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

    double throttle_command_with_antiwindup(
      const double energy_error,
      const double dt,
      double& energy_error_integral,
      const double ki,
      const double feedforward,
      const double kp,
      const double va,
      const double tmax)
    {
      const double integral_candidate = energy_error_integral + energy_error * dt;
      const double command_unsaturated =
        (ki * integral_candidate + (feedforward - kp * energy_error) / va) / tmax;
      const double command = std::clamp(command_unsaturated, 0.0, 1.0);

      const bool saturated_high = command_unsaturated > 1.0;
      const bool saturated_low = command_unsaturated < 0.0;
      const bool drives_out_of_high_saturation = saturated_high && energy_error < 0.0;
      const bool drives_out_of_low_saturation = saturated_low && energy_error > 0.0;

      if ((!saturated_high && !saturated_low) ||
          drives_out_of_high_saturation ||
          drives_out_of_low_saturation) {
        energy_error_integral = integral_candidate;
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
  
  LOS4ControllerMavlink::~LOS4ControllerMavlink() {}

  void LOS4ControllerMavlink::initialize() {
    // Load Mass
    node_->declare_parameter<double>("controllers.los4controller.m", 1.5);
    node_->declare_parameter<double>("controllers.los4controller.Tmax", 15.0);

    // Load Gains
    node_->declare_parameter<double>("controllers.los4controller.gains.k1", 1.0);
    node_->declare_parameter<double>("controllers.los4controller.gains.k2", 1.0);
    node_->declare_parameter<double>("controllers.los4controller.gains.kpE", 1.0);
    node_->declare_parameter<double>("controllers.los4controller.gains.kiE", 0.0);
    node_->declare_parameter<double>("controllers.los4controller.gains.kpB", 1.0);
    node_->declare_parameter<double>("controllers.los4controller.gains.kiB", 0.0);
    node_->declare_parameter<double>("controllers.los4controller.gains.ka", 1.0);
    node_->declare_parameter<double>("controllers.los4controller.gains.kphi", 1.0);
    node_->declare_parameter<double>("controllers.los4controller.gains.ktheta", 1.0);
    node_->declare_parameter<double>("controllers.los4controller.limits.throttle_min", 0.0);
    node_->declare_parameter<double>("controllers.los4controller.limits.throttle_max", 1.0);
    node_->declare_parameter<double>("controllers.los4controller.limits.pitch_min", -PI);
    node_->declare_parameter<double>("controllers.los4controller.limits.pitch_max", PI);
    node_->declare_parameter<double>("controllers.los4controller.limits.lateral_acceleration_min", -4.0);
    node_->declare_parameter<double>("controllers.los4controller.limits.lateral_acceleration_max", 4.0);

    m_ = node_->get_parameter("controllers.los4controller.m").as_double();
    Tmax_ = node_->get_parameter("controllers.los4controller.Tmax").as_double();
    k1_ = node_->get_parameter("controllers.los4controller.gains.k1").as_double();
    k2_ = node_->get_parameter("controllers.los4controller.gains.k2").as_double();
    kpE_ = node_->get_parameter("controllers.los4controller.gains.kpE").as_double();
    kiE_ = node_->get_parameter("controllers.los4controller.gains.kiE").as_double();
    kpB_ = node_->get_parameter("controllers.los4controller.gains.kpB").as_double();
    kiB_ = node_->get_parameter("controllers.los4controller.gains.kiB").as_double();
    ka_ = node_->get_parameter("controllers.los4controller.gains.ka").as_double();
    kphi_ = node_->get_parameter("controllers.los4controller.gains.kphi").as_double();
    ktheta_ = node_->get_parameter("controllers.los4controller.gains.ktheta").as_double();
    throttle_min_ = node_->get_parameter("controllers.los4controller.limits.throttle_min").as_double();
    throttle_max_ = node_->get_parameter("controllers.los4controller.limits.throttle_max").as_double();
    pitch_min_ = node_->get_parameter("controllers.los4controller.limits.pitch_min").as_double();
    pitch_max_ = node_->get_parameter("controllers.los4controller.limits.pitch_max").as_double();
    lateral_acceleration_min_ = node_->get_parameter("controllers.los4controller.limits.lateral_acceleration_min").as_double();
    lateral_acceleration_max_ = node_->get_parameter("controllers.los4controller.limits.lateral_acceleration_max").as_double();

    if (throttle_min_ > throttle_max_) {
      std::swap(throttle_min_, throttle_max_);
    }
    if (Tmax_ <= 0.0) {
      RCLCPP_WARN_STREAM(
        node_->get_logger(),
        "LOS4ControllerMavlink Tmax must be positive, using default Tmax = 15.0");
      Tmax_ = 15.0;
    }
    if (pitch_min_ > pitch_max_) {
      std::swap(pitch_min_, pitch_max_);
    }
    if (lateral_acceleration_min_ > lateral_acceleration_max_) {
      std::swap(lateral_acceleration_min_, lateral_acceleration_max_);
    }

    // Log Gains 
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink vehicle mass: m = " << m_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink max thrust: Tmax = " << Tmax_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink vehicle gain: k1 = " << k1_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink vehicle gain: k2 = " << k2_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink vehicle gain: kpE = " << kpE_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink vehicle gain: kiE = " << kiE_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink vehicle gain: kpB = " << kpB_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink vehicle gain: kiB = " << kiB_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink vehicle gain: ka = " << ka_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink vehicle gain: kphi = " << kphi_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink vehicle gain: ktheta = " << ktheta_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink throttle limits: [" << throttle_min_ << ", " << throttle_max_ << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink pitch limits: [" << pitch_min_ << ", " << pitch_max_ << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "LOS4ControllerMavlink lateral acceleration limits: [" << lateral_acceleration_min_ << ", " << lateral_acceleration_max_ << "]");

    // Log that the LOS4ControllerMavlink was initialized
    RCLCPP_INFO(node_->get_logger(), "LOS4ControllerMavlink initialized");
  }

  void LOS4ControllerMavlink::set_position(const double dt, const Eigen::Vector3d& p) {
    (void)dt;
    (void)p;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "LOS4ControllerMavlink does not support position control");
  }

  void LOS4ControllerMavlink::set_attitude(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v) {
    (void)dt;
    (void)p;
    (void)v;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "LOS4ControllerMavlink does not support attitude control");
  }

  void LOS4ControllerMavlink::set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) {
    if (!std::isfinite(dt) || dt <= 0.0 || !p.allFinite() ||
        !v.allFinite() || !eta.allFinite() || v.head<2>().norm() < 1e-6) {
      throw std::runtime_error("LOS4 requires positive dt and finite moving-flight state");
    }
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
      static_cast<float>(ka_),
      static_cast<float>(kphi_),
      static_cast<float>(ktheta_)});

    const double h_d_dot = -Va * h(2);

    if (!z_ref_initialized_) {
      z_ref_ = -(p.z());
      z_ref_initialized_ = true;
    }

    z_ref_ += h_d_dot * dt;

    // Longitudinal Controller
    constexpr double g = 9.81;
    const double K_err = 0.5 * m_ * (Va*Va - v.squaredNorm());
    const double U_err = m_ * g * (-p.z() - z_ref_);

    const double E_err = U_err + K_err;
    const double B_err = -(U_err - K_err);

    const double throttle_cmd = throttle_command_with_antiwindup(
      E_err,
      dt,
      E_err_int_,
      kiE_,
      m_ * g * h_d_dot,
      kpE_,
      Va,
      Tmax_);

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
    const double roll_cmd = std::atan(lateral_acceleration_cmd / g);

    // Attitude Controller
    if (!attitude_cmd_initialized_) {
      previous_roll_cmd_ = roll_cmd;
      previous_pitch_cmd_ = pitch_cmd;
      attitude_cmd_initialized_ = true;
    }

    const double roll_cmd_dot = (roll_cmd - previous_roll_cmd_) / dt;
    const double pitch_cmd_dot = (pitch_cmd - previous_pitch_cmd_) / dt;

    previous_roll_cmd_ = roll_cmd;
    previous_pitch_cmd_ = pitch_cmd;

    const double roll_rate_cmd = roll_cmd_dot + kphi_ * (roll_cmd - eta(0));
    const double pitch_rate_cmd = pitch_cmd_dot + ktheta_ * (pitch_cmd - eta(1));
    const double yaw_rate_cmd = (g*std::tan(roll_cmd)) / (Va*std::cos(pitch_cmd));

    const double p_cmd = roll_rate_cmd - yaw_rate_cmd*std::sin(eta(1));
    const double q_cmd = pitch_rate_cmd*std::cos(eta(0)) + yaw_rate_cmd*std::sin(eta(0))*std::cos(eta(1));
    const double r_cmd = -pitch_rate_cmd*std::sin(eta(0)) + yaw_rate_cmd*std::cos(eta(0))*std::cos(eta(1));

    /*
    // Log command
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      500,
      "LOS4 commands: throttle=%.3f, lateral_acceleration=%.3f, lateral_ff=%.3f, roll_rate=%.3f, pitch_rate=%.3f, yaw_rate=%.3f, h_ref=%.3f",
      throttle_cmd,
      lateral_acceleration_cmd,
      lateral_acceleration_feedforward,
      p_cmd,
      q_cmd,
      r_cmd,
      z_ref_);*/

    // Same FRD rates and forward throttle as the DDS controller. MAVSDK
    // accepts degrees/s; PX4 maps scalar thrust to body X for fixed-wing.
    if (!std::isfinite(p_cmd) || !std::isfinite(q_cmd) ||
        !std::isfinite(r_cmd) || !std::isfinite(throttle_cmd)) {
      throw std::runtime_error("Non-finite LOS4 rate/throttle command");
    }
    const auto result = offboard_->set_attitude_rate({
      static_cast<float>(p_cmd * 180.0 / PI),
      static_cast<float>(q_cmd * 180.0 / PI),
      static_cast<float>(r_cmd * 180.0 / PI),
      static_cast<float>(throttle_cmd)});
    if (result != mavsdk::Offboard::Result::Success) {
      throw std::runtime_error("MAVSDK rejected LOS4 body-rate setpoint");
    }
  }

  void LOS4ControllerMavlink::set_path(const int type, const double* path) {
    // Validate before resetting state so a rejected path cannot replace an
    // active path and leave MAVSDK retransmitting its previous command.
    if (type < 1 || type > 3) {
      throw std::invalid_argument("LOS4 supports line, circle and lemniscate paths, not waypoints");
    }
    const int size = type == 1 ? 7 : 5;
    for (int i = 0; i < size; ++i) {
      if (!std::isfinite(path[i])) throw std::invalid_argument("Non-finite LOS4 path");
    }
    if (path[size - 1] <= 0.0 ||
        (type == 1 && std::hypot(path[3] - path[0], path[4] - path[1]) < 1e-6) ||
        (type != 1 && std::abs(path[3]) < 1e-6)) {
      throw std::invalid_argument("LOS4 requires positive speed and nonzero horizontal path geometry");
    }
    path_.type = type;
    gamma_ = 0.0;
    z_ref_initialized_ = false;
    attitude_cmd_initialized_ = false;
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
    
    // AutopilotMavlink primes setpoints before entering Offboard.
  }
}
