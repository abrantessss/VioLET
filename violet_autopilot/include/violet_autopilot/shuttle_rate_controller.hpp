#pragma once

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"

#include <controller.hpp>

namespace autopilot {
  class ShuttleRateController : public autopilot::Controller {
    public:
      ~ShuttleRateController();

      void initialize() override;

      void set_position(const double dt, const Eigen::Vector3d& p) override;

      void set_attitude(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v) override;

      void set_attitude_rate(
        const double dt,
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& v,
        const Eigen::Vector3d& eta) override;

      void set_path(const int type, const double* path) override;
      void reset() override;

    protected:
      void on_rate_setpoint_callback(const px4_msgs::msg::VehicleRatesSetpoint::ConstSharedPtr msg);
      void publish_rate_setpoint();
      void publish_offboard_mode();
      void publish_offboard_request();

      px4_msgs::msg::VehicleRatesSetpoint rates_msg_;
      px4_msgs::msg::OffboardControlMode offboard_msg_;

      rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_pub_;
      rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
      rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr mode_pub_;
      rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rate_setpoint_sub_;
      rclcpp::TimerBase::SharedPtr setpoint_timer_;

      double publish_rate_hz_{50.0};
      bool have_rate_setpoint_{false};
  };
}
