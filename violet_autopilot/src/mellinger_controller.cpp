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

  void MellingerController::set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) {
    double gamma_dot = 0.0;
    double gamma_ddot = 0.0;
    double gamma_dddot = 0.0;
    Eigen::Vector3d pd = Eigen::Vector3d::Zero();
    Eigen::Vector3d pdd = Eigen::Vector3d::Zero();
    Eigen::Vector3d pddd = Eigen::Vector3d::Zero();
    Eigen::Vector3d pdddd = Eigen::Vector3d::Zero();
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
      const double vd = path_.line_v;
      Eigen::Vector3d p0 = path_.line_p0;
      Eigen::Vector3d p1 = path_.line_p1;

      pd = p0 + gamma_ * (p1 - p0);
    
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

      if (gamma_ >= 1.0) {
        gamma_ = 1.0;
        gamma_dot = 0.0;
      }

      pdd = (p1 - p0) * gamma_dot;
      pddd = (p1 - p0) * gamma_ddot;
      pdddd = (p1 - p0) * gamma_dddot;
    }
    else if (path_.type == 2) {
      // Circle
      const double vd = path_.circle_v;
      const double r = path_.circle_R;
      const Eigen::Vector3d& c = path_.circle_c;

      pd << c.x() + r * std::cos(gamma_),
            c.y() + r * std::sin(gamma_),
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

      pdd <<  -r*gamma_dot*std::sin(gamma_),
              r*gamma_dot*std::cos(gamma_),
              0;

      pddd << -r*(std::cos(gamma_)*std::pow(gamma_dot, 2) + std::sin(gamma_)*gamma_ddot),
              -r*(std::sin(gamma_)*std::pow(gamma_dot, 2) - std::cos(gamma_)*gamma_ddot),
              0;

      pdddd <<  r*(std::sin(gamma_)*std::pow(gamma_dot, 3) - 3*std::cos(gamma_)*gamma_dot*gamma_ddot - std::sin(gamma_)*gamma_dddot),
                r*(-std::cos(gamma_)*std::pow(gamma_dot, 3) - 3*std::sin(gamma_)*gamma_dot*gamma_ddot + std::cos(gamma_)*gamma_dddot),
                0;

    }
    else {      // Lemniscate
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

      pdd  << (a*std::sin(gamma_)*(std::pow(std::sin(gamma_), 2)-3)/std::pow((1+std::pow(std::sin(gamma_), 2)), 2))*gamma_dot,
              (a*(1-3*std::pow(std::sin(gamma_), 2))/std::pow((1+std::pow(std::sin(gamma_),2)),2))*gamma_dot,
              0;

      pddd << (a * std::cos(gamma_) * (-std::pow(std::sin(gamma_),4) + 12*std::pow(std::sin(gamma_),2) - 3) / std::pow(1 + std::pow(std::sin(gamma_),2), 3)) * std::pow(gamma_dot,2) + (a * std::sin(gamma_) * (std::pow(std::sin(gamma_),2) - 3) / std::pow(1 + std::pow(std::sin(gamma_),2), 2)) * gamma_ddot,
              (a * 2*std::sin(gamma_)*std::cos(gamma_) * (3*std::pow(std::sin(gamma_),2) - 5) / std::pow(1 + std::pow(std::sin(gamma_),2), 3)) * std::pow(gamma_dot,2) + (a * (1 - 3*std::pow(std::sin(gamma_),2)) / std::pow(1 + std::pow(std::sin(gamma_),2), 2)) * gamma_ddot,
              0;

      pdddd << (a * std::sin(gamma_) * (-std::pow(std::sin(gamma_),6) + 43*std::pow(std::sin(gamma_),4) - 103*std::pow(std::sin(gamma_),2) + 45) / std::pow(1 + std::pow(std::sin(gamma_),2),4)) * std::pow(gamma_dot,3) + 3 * (a * std::cos(gamma_) * (-std::pow(std::sin(gamma_),4) + 12*std::pow(std::sin(gamma_),2) - 3) / std::pow(1 + std::pow(std::sin(gamma_),2),3)) * gamma_dot * gamma_ddot + (a * std::sin(gamma_) * (std::pow(std::sin(gamma_),2) - 3) / std::pow(1 + std::pow(std::sin(gamma_),2),2)) * gamma_dddot,
               (2*a * (6*std::pow(std::sin(gamma_),6) - 41*std::pow(std::sin(gamma_),4) + 44*std::pow(std::sin(gamma_),2) - 5) / std::pow(1 + std::pow(std::sin(gamma_),2),4)) * std::pow(gamma_dot,3) + 3 * (a * 2*std::sin(gamma_)*std::cos(gamma_) * (3*std::pow(std::sin(gamma_),2) - 5) / std::pow(1 + std::pow(std::sin(gamma_),2),3)) * gamma_dot * gamma_ddot + (a * (1 - 3*std::pow(std::sin(gamma_),2)) / std::pow(1 + std::pow(std::sin(gamma_),2),2)) * gamma_dddot,
               0;
    }
    
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

    Eigen::Vector3d wd;
    wd[0] = mass_ / T * Ybd.dot(pdddd);
    wd[1] = -mass_ / T * Xbd.dot(pdddd);
    wd[2] = 0;

    Eigen::Matrix3d Re = (Rd.transpose() * R) - (R.transpose() * Rd);

    // Compute the vee map of the rotation error and project into the coordinates of the manifold
    Eigen::Vector3d eR;
    eR << -Re(1,2), Re(0, 2), -Re(0,1);
    eR = 0.5*eR;

    // Get attitude control law
    Eigen::Vector3d attitude_rate = wd - (kr_ * eR);
    

    const uint64_t now_us = node_->get_clock()->now().nanoseconds() / 1000;

    rates_msg_.timestamp = now_us;
    
    rates_msg_.roll = attitude_rate[0]; 
    rates_msg_.pitch = attitude_rate[1];
    rates_msg_.yaw = attitude_rate[2];
    rates_msg_.thrust_body[0] = 0.0f;
    rates_msg_.thrust_body[1] = 0.0f;
    rates_msg_.thrust_body[2] = static_cast<float>(-std::clamp(T / 35, 0.0, 1.0));
  
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
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    mode_pub_->publish(msg);
  }
}
