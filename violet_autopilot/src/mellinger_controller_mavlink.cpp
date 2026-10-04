#include "mellinger_controller_mavlink.hpp"

#include <algorithm>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autopilot {
  
  MellingerControllerMavlink::~MellingerControllerMavlink() {}

  void MellingerControllerMavlink::initialize() {
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

    if (kp.size() != 3 || kd.size() != 3 || kr.size() != 3 ||
        !std::isfinite(mass_) || mass_ <= 0.0) {
      throw std::invalid_argument("Mellinger requires positive mass and three gains per axis group");
    }
    for (const auto& gains : {kp, kd, kr}) {
      for (double gain : gains) {
        if (!std::isfinite(gain)) throw std::invalid_argument("Non-finite Mellinger gain");
      }
    }

    kp_ = Eigen::Matrix3d::Identity();
    kd_ = Eigen::Matrix3d::Identity();
    kr_ = Eigen::Matrix3d::Identity();
    for(unsigned int i=0; i < 3; i++) {
      kp_(i, i) = kp[i];
      kd_(i, i) = kd[i];
      kr_(i, i) = kr[i];
    }

    // Log the gains
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerControllerMavlink vehicle mass: m = " << mass_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerControllerMavlink gains: kp = [" << kp[0] << ", " << kp[1] << ", " << kp[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerControllerMavlink gains: kd = [" << kd[0] << ", " << kd[1] << ", " << kd[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerControllerMavlink gains: kr = [" << kr[0] << ", " << kr[1] << ", " << kr[2] << "]");

    // Log that the MellingerControllerMavlink was initialized
    RCLCPP_INFO(node_->get_logger(), "MellingerControllerMavlink initialized");
  }

  void MellingerControllerMavlink::set_position(const double dt, const Eigen::Vector3d& p) {
    (void)dt;
    (void)p;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "MellingerControllerMavlink does not support position setpoints");
  }

  void MellingerControllerMavlink::set_attitude(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v) {
    (void)dt;
    (void)p;
    (void)v;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "MellingerControllerMavlink does not support attitude control");
  }

  void MellingerControllerMavlink::set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) {
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
    Eigen::Vector3d Yc = Eigen::Vector3d(0.0, 1.0, 0.0); //eta_d [0 0 0]'
    Eigen::Vector3d Zbd = -Fd/Fd.norm();
    Eigen::Vector3d Xbd = Yc.cross(Zbd);
    Xbd = Xbd/Xbd.norm();
    Eigen::Vector3d Ybd = Zbd.cross(Xbd);
    Ybd = Ybd/Ybd.norm();

    Eigen::Matrix3d Rd;
    Rd.col(0) = Xbd;
    Rd.col(1) = Ybd;
    Rd.col(2) = Zbd;

    Eigen::Vector3d Ycd;
    Ycd << -std::cos(0.0), -std::sin(0.0), 0.0;
    Ycd *= 0;

    Eigen::Vector3d az = mass_ * (g_ - u);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Eigen::Vector3d Zbdd = -mass_ / az.norm() * (I - (az * az.transpose()) / az.squaredNorm()) * pdddd;

    Eigen::Vector3d ax = Yc.cross(Zbd);
    Eigen::Vector3d Xbdd = 1.0 / ax.norm() * (I - (ax * ax.transpose()) / ax.squaredNorm()) * (Ycd.cross(Zbd) + Yc.cross(Zbdd));

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
    

    // PX4 and MAVSDK use FRD body axes. MAVSDK takes degrees/s and
    // positive collective thrust, whereas the DDS setpoint uses negative body Z.
    if (!attitude_rate.allFinite() || !std::isfinite(T)) {
      throw std::runtime_error("Non-finite Mellinger rate/thrust command");
    }
    constexpr double rad_to_deg = 180.0 / 3.14159265358979323846;
    const auto result = offboard_->set_attitude_rate({
      static_cast<float>(attitude_rate[0] * rad_to_deg),
      static_cast<float>(attitude_rate[1] * rad_to_deg),
      static_cast<float>(attitude_rate[2] * rad_to_deg),
      static_cast<float>(std::clamp(T / 134.0, 0.0, 1.0))});
    if (result != mavsdk::Offboard::Result::Success) {
      throw std::runtime_error("MAVSDK rejected body-rate setpoint");
    }
  }

  void MellingerControllerMavlink::set_path(const int type, const double* path) {
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

    // AutopilotMavlink primes setpoints before requesting Offboard mode.
  }
}
