#pragma once 

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/fixed_wing_longitudinal_setpoint.hpp"
#include "px4_msgs/msg/fixed_wing_lateral_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

#include <controller.hpp>

namespace autopilot{
  class LOS3Controller : public autopilot::Controller {
    public:
      ~LOS3Controller();

      void initialize() override;

      void set_position(const double dt, const Eigen::Vector3d& p) override;

      void set_attitude(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v) override;

      void set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) override;

      void set_path(const int type, const double* path) override;
    
    protected:
      // ROS2 messages
      px4_msgs::msg::FixedWingLongitudinalSetpoint lon_msg_;
      px4_msgs::msg::FixedWingLateralSetpoint lat_msg_;
      px4_msgs::msg::OffboardControlMode offboard_msg_;

      // ROS2 publishers
      rclcpp::Publisher<px4_msgs::msg::FixedWingLongitudinalSetpoint>::SharedPtr lon_pub_;
      rclcpp::Publisher<px4_msgs::msg::FixedWingLateralSetpoint>::SharedPtr lat_pub_;
      rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
      rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr mode_pub_;

      // Variables
      double gamma_;
      double m_;
      double Tmax_;
      double k1_;
      double k2_;
      double kpE_;
      double kiE_;
      double kpB_;
      double kiB_;
      double ka_;
      double throttle_min_;
      double throttle_max_;
      double pitch_min_;
      double pitch_max_;
      double lateral_acceleration_min_;
      double lateral_acceleration_max_;
      double z_ref_;
      double E_err_int_{0.0};
      double B_err_int_{0.0};
      bool z_ref_initialized_{false};
  };
}
