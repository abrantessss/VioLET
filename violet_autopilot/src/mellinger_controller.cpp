#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "mellinger_controller.hpp"

namespace autopilot {
  
  MellingerController::~MellingerController() {}

  void MellingerController::initialize() {
    node_->declare_parameter<std::string>("controllers.mellingercontroller.publishers.setpoint", "fmu/in/vehicle_rates_setpoint");
    rates_pub_ = node_->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(node_->get_parameter("controllers.mellingercontroller.publishers.setpoint").as_string(), rclcpp::SensorDataQoS());
    
    node_->declare_parameter<std::string>("publishers.mode.offboard", "fmu/in/offboard_control_mode");
    offboard_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(node_->get_parameter("publishers.mode.offboard").as_string(), rclcpp::SensorDataQoS());
    
    node_->declare_parameter<std::string>("publishers.mode.request", "fmu/in/vehicle_command");
    mode_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(node_->get_parameter("publishers.mode.request").as_string(), rclcpp::SensorDataQoS());

    node_->declare_parameter<double>("controllers.mellingercontroller.kphi", 0.0005);
    if (!std::isfinite(node_->get_parameter("controllers.mellingercontroller.kphi").as_double()) ||
        node_->get_parameter("controllers.mellingercontroller.kphi").as_double() < 0.0) {
      throw std::invalid_argument("Mellinger kphi must be finite and nonnegative");
    }
    // Load Gains and Dynamics
    node_->declare_parameter<double>("controllers.mellingercontroller.mass", 1.0);
    node_->declare_parameter<std::vector<double>>("controllers.mellingercontroller.gains.kp", {1.0, 1.0, 1.0});
    node_->declare_parameter<std::vector<double>>("controllers.mellingercontroller.gains.kd", {1.0, 1.0, 1.0});
    node_->declare_parameter<std::vector<double>>("controllers.mellingercontroller.gains.kr", {1.0, 1.0, 1.0});

    mass_ = node_->get_parameter("controllers.mellingercontroller.mass").as_double();
    auto kp = node_->get_parameter("controllers.mellingercontroller.gains.kp").as_double_array();
    auto kd = node_->get_parameter("controllers.mellingercontroller.gains.kd").as_double_array();
    auto kr = node_->get_parameter("controllers.mellingercontroller.gains.kr").as_double_array();

    kp_ = Eigen::Matrix3d::Identity();
    kd_ = Eigen::Matrix3d::Identity();
    kr_ = Eigen::Matrix3d::Identity();
    for(unsigned int i=0; i < 3; i++) {
      kp_(i, i) = kp[i];
      kd_(i, i) = kd[i];
      kr_(i, i) = kr[i];
    }

    // Log the gains
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerController vehicle mass: m = " << mass_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerController gains: kp = [" << kp[0] << ", " << kp[1] << ", " << kp[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerController gains: kd = [" << kd[0] << ", " << kd[1] << ", " << kd[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerController gains: kr = [" << kr[0] << ", " << kr[1] << ", " << kr[2] << "]");

    // Log that the MellingerController was initialized
    RCLCPP_INFO(node_->get_logger(), "MellingerController initialized");
  }

  void MellingerController::set_position(const double dt, const Eigen::Vector3d& p) {
    (void)dt;
    (void)p;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "MellingerController does not support position setpoints");
  }

  void MellingerController::set_attitude(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v) {
    (void)dt;
    (void)p;
    (void)v;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "MellingerController does not support attitude control");
  }

  void MellingerController::set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) {
    const double gamma_at_evaluation = gamma_;
    // Evaluate the entire reference at the current phase. Advance only for the next cycle.
    double gamma_dot = 0.0;
    double gamma_ddot = 0.0;
    double gamma_dddot = 0.0;
    //constexpr double kMinDpdDgammaNorm = 1e-6;
    const double k = node_->get_parameter("controllers.mellingercontroller.kphi").as_double();
    double vd = 0.0;
    Eigen::Vector3d pd = Eigen::Vector3d::Zero();
    Eigen::Vector3d pdd = Eigen::Vector3d::Zero();
    Eigen::Vector3d pddd = Eigen::Vector3d::Zero();
    Eigen::Vector3d pdddd = Eigen::Vector3d::Zero();
    Eigen::Vector3d dpd_dgamma = Eigen::Vector3d::Zero();
    Eigen::Vector3d ep = Eigen::Vector3d::Zero();
    
    Eigen::Matrix3d R =( 
      Eigen::AngleAxisd(eta(2), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(eta(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(eta(0), Eigen::Vector3d::UnitX())
    ).toRotationMatrix();
    
    if (path_.type == 0) {
      // Waypoint
      pd = path_.waypoint;
      ep = p - pd;
    }
    else if (path_.type == 1) {
      // Line
      vd = path_.line_v;
      Eigen::Vector3d p0 = path_.line_p0;
      Eigen::Vector3d p1 = path_.line_p1;
      dpd_dgamma = p1 - p0;
      //const double dpd_dgamma_norm = std::max(dpd_dgamma.norm(), kMinDpdDgammaNorm);

      pd = p0 + gamma_at_evaluation * dpd_dgamma;
    
      ep = p - pd;

      const double ep_norm = ep.norm();
      gamma_dot = vd * std::exp(-k * std::pow(ep_norm,2));

      Eigen::Vector3d ep_dot = v - gamma_dot * dpd_dgamma;
      gamma_ddot = -2.0 * k * gamma_dot * ep.dot(ep_dot);
      gamma_dddot = -2.0 * k * (gamma_ddot * ep.dot(ep_dot)
        + gamma_dot * ep_dot.squaredNorm());

      if (gamma_at_evaluation >= 1.0) {
        gamma_dot = 0.0;
        gamma_ddot = 0.0;
        gamma_dddot = 0.0;
      }

      pdd = dpd_dgamma * gamma_dot;
      pddd = dpd_dgamma * gamma_ddot;
      pdddd = dpd_dgamma * gamma_dddot;
    }
    else if (path_.type == 2) {
      // Circle
      vd = path_.circle_v;
      const double r = path_.circle_R;
      const Eigen::Vector3d& c = path_.circle_c;

      pd << c.x() + r * std::cos(gamma_at_evaluation),
            c.y() + r * std::sin(gamma_at_evaluation),
            c.z();

      ep = p - pd;

      dpd_dgamma << -r * std::sin(gamma_at_evaluation),
                     r * std::cos(gamma_at_evaluation),
                     0.0;
      //const double dpd_dgamma_norm = std::max(std::abs(r), kMinDpdDgammaNorm);
      const double ep_norm = ep.norm();
      gamma_dot = vd * std::exp(-k * std::pow(ep_norm,2));

      Eigen::Vector3d ep_dot = v - gamma_dot * dpd_dgamma;
      gamma_ddot = -2.0 * k * gamma_dot * ep.dot(ep_dot);
      gamma_dddot = -2.0 * k * (gamma_ddot * ep.dot(ep_dot)
        + gamma_dot * ep_dot.squaredNorm());

      pdd <<  -r*gamma_dot*std::sin(gamma_at_evaluation),
              r*gamma_dot*std::cos(gamma_at_evaluation),
              0;

      pddd << -r*(std::cos(gamma_at_evaluation)*std::pow(gamma_dot, 2) + std::sin(gamma_at_evaluation)*gamma_ddot),
              -r*(std::sin(gamma_at_evaluation)*std::pow(gamma_dot, 2) - std::cos(gamma_at_evaluation)*gamma_ddot),
              0;

      pdddd <<  r*(std::sin(gamma_at_evaluation)*std::pow(gamma_dot, 3) - 3*std::cos(gamma_at_evaluation)*gamma_dot*gamma_ddot - std::sin(gamma_at_evaluation)*gamma_dddot),
                r*(-std::cos(gamma_at_evaluation)*std::pow(gamma_dot, 3) - 3*std::sin(gamma_at_evaluation)*gamma_dot*gamma_ddot + std::cos(gamma_at_evaluation)*gamma_dddot),
                0;

    }
    else {
      // Gerono lemniscate: x = a*cos(gamma), y = a*sin(gamma)*cos(gamma).
      vd = path_.lemniscate_v;
      const double a = path_.lemniscate_a;
      const Eigen::Vector3d& c = path_.lemniscate_c;
      const double s = std::sin(gamma_at_evaluation);
      const double cg = std::cos(gamma_at_evaluation);
      const double s2 = std::sin(2.0 * gamma_at_evaluation);
      const double c2 = std::cos(2.0 * gamma_at_evaluation);

      pd << c.x() + a * cg, c.y() + a * s * cg, c.z();
      ep = p - pd;

      dpd_dgamma << -a * s, a * c2, 0.0;
      const Eigen::Vector3d d2pd_dgamma2(-a * cg, -2.0 * a * s2, 0.0);
      const Eigen::Vector3d d3pd_dgamma3(a * s, -4.0 * a * c2, 0.0);

      gamma_dot = vd * std::exp(-k * ep.squaredNorm());
      const Eigen::Vector3d ep_dot = v - gamma_dot * dpd_dgamma;
      gamma_ddot = -2.0 * k * gamma_dot * ep.dot(ep_dot);
      gamma_dddot = -2.0 * k * (gamma_ddot * ep.dot(ep_dot)
        + gamma_dot * ep_dot.squaredNorm());

      pdd = dpd_dgamma * gamma_dot;
      pddd = d2pd_dgamma2 * (gamma_dot * gamma_dot) + dpd_dgamma * gamma_ddot;
      pdddd = d3pd_dgamma3 * (gamma_dot * gamma_dot * gamma_dot)
        + 3.0 * d2pd_dgamma2 * gamma_dot * gamma_ddot
        + dpd_dgamma * gamma_dddot;
    }

    gamma_ = gamma_at_evaluation + gamma_dot * dt;
    if (path_.type == 1) {
      gamma_ = std::min(gamma_, 1.0);
    }

    publish_plot_data(
      gamma_dot,
      vd,
      pd,
      dpd_dgamma,
      {
        static_cast<float>(kp_(0, 0)),
        static_cast<float>(kp_(1, 1)),
        static_cast<float>(kp_(2, 2)),
        static_cast<float>(kd_(0, 0)),
        static_cast<float>(kd_(1, 1)),
        static_cast<float>(kd_(2, 2)),
        static_cast<float>(kr_(0, 0)),
        static_cast<float>(kr_(1, 1)),
        static_cast<float>(kr_(2, 2)),
      });
    
    // Compute velocity error
    Eigen::Vector3d ev = v - pdd;

    // Get translational control law
    Eigen::Vector3d u = -kp_*ep - kd_*ev + pddd;

        // Calculate thrust
    Eigen::Vector3d Fd = mass_ * (u - g_);
    double T = -Fd.dot(R.col(2));

    // Calculate desired rotation
    constexpr double kMinNorm = 1e-6;

    Eigen::Vector3d Yc = Eigen::Vector3d(0.0, 1.0, 0.0); //eta_d [0 0 0]'
    const double Fd_norm = std::max(Fd.norm(), kMinNorm);
    Eigen::Vector3d Zbd = -Fd / Fd_norm;

    Eigen::Vector3d Xbd = Yc.cross(Zbd);
    const double Xbd_norm = std::max(Xbd.norm(), kMinNorm);
    Xbd = Xbd / Xbd_norm;

    Eigen::Vector3d Ybd = Zbd.cross(Xbd);
    const double Ybd_norm = std::max(Ybd.norm(), kMinNorm);
    Ybd = Ybd / Ybd_norm;

    Eigen::Matrix3d Rd;
    Rd.col(0) = Xbd;
    Rd.col(1) = Ybd;
    Rd.col(2) = Zbd;

    Eigen::Vector3d Ycd;
    Ycd << -std::cos(0.0), -std::sin(0.0), 0.0;
    Ycd *= 0;

    Eigen::Vector3d az = mass_ * (g_ - u);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    const double az_norm = std::max(az.norm(), kMinNorm);
    const double az_sqnorm = az_norm * az_norm;
    Eigen::Vector3d Zbdd = -mass_ / az_norm * (I - (az * az.transpose()) / az_sqnorm) * pdddd;

    Eigen::Vector3d ax = Yc.cross(Zbd);
    const double ax_norm = std::max(ax.norm(), kMinNorm);
    const double ax_sqnorm = ax_norm * ax_norm;
    Eigen::Vector3d Xbdd = 1.0 / ax_norm * (I - (ax * ax.transpose()) / ax_sqnorm) * (Ycd.cross(Zbd) + Yc.cross(Zbdd));

    Eigen::Vector3d Ybdd = Zbdd.cross(Xbd) + Zbd.cross(Xbdd);

    Eigen::Matrix3d Rdd;
    Rdd.col(0) = Xbdd;
    Rdd.col(1) = Ybdd;
    Rdd.col(2) = Zbdd;

    // vee map
    Eigen::Matrix3d Omega_d = Rd.transpose() * Rdd;

    Eigen::Vector3d wd;
    wd << Omega_d(2,1), Omega_d(0,2), Omega_d(1,0);

    Eigen::Matrix3d Re = (Rd.transpose() * R) - (R.transpose() * Rd);

    // Compute the vee map of the rotation error and project into the coordinates of the manifold
    Eigen::Vector3d eR;
    eR << -Re(1,2), Re(0, 2), -Re(0,1);
    eR = 0.5*eR;

    publish_mellinger_results(gamma_at_evaluation, gamma_dot, gamma_ddot, gamma_dddot,
      vd, k, p, v, pd, pdd, pddd, pdddd, dpd_dgamma, ep, R, Rd, eR);

    // Get attitude control law
    Eigen::Vector3d attitude_rate = wd - (kr_ * eR);
    

    const uint64_t now_us = node_->get_clock()->now().nanoseconds() / 1000;

    rates_msg_.timestamp = now_us;
    
    rates_msg_.roll = attitude_rate[0]; 
    rates_msg_.pitch = attitude_rate[1];
    rates_msg_.yaw = attitude_rate[2];
    rates_msg_.thrust_body[0] = 0.0f;
    rates_msg_.thrust_body[1] = 0.0f;
    const double kf = 1.709716e-05;          // motorConstant
    const double omega = std::sqrt(std::max(T, 0.0) / (4.0 * kf));
    const double cmd = std::clamp((omega - 100.0) / 1400.0, 0.0, 1.0);
    rates_msg_.thrust_body[2] = static_cast<float>(-cmd);
  
    rates_pub_->publish(rates_msg_);

    offboard_msg_.timestamp = now_us;
    offboard_msg_.position = false;
    offboard_msg_.velocity = false;
    offboard_msg_.acceleration = false;
    offboard_msg_.attitude = false;
    offboard_msg_.body_rate = true;
    offboard_msg_.thrust_and_torque = false;
    offboard_msg_.direct_actuator = false;

    offboard_pub_->publish(offboard_msg_);
  }

  void MellingerController::set_path(const int type, const double* path) {
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
    msg.target_system = vehicle_id_;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    mode_pub_->publish(msg);
  }
}
