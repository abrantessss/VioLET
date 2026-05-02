#pragma once 

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/fixed_wing_longitudinal_setpoint.hpp"
#include "px4_msgs/msg/fixed_wing_lateral_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

#include <controller.hpp>

namespace autopilot{
  class LOSController : public autopilot::Controller {
    public:
      ~LOSController();

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
      double k1_;
      double k2_;
  };
}
