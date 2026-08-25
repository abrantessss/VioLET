#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "controller.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "rate_controller.hpp"

namespace autopilot {
  class ShuttleRateController : public autopilot::Controller {
    public:
      using Controller::Controller;
      ~ShuttleRateController() override;

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

    private:
      void on_rate_setpoint_callback(const px4_msgs::msg::VehicleRatesSetpoint::ConstSharedPtr msg);
      void on_force_setpoint_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg);
      void update_rate_controller(double dt);
      void publish_actuators();
      void publish_offboard_heartbeat();
      void publish_offboard_mode();
      void publish_offboard_request(
        const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr& publisher,
        int target_system);
      void publish_wrench(
        const Eigen::Vector3d& torque_sp_Nm,
        const Eigen::Vector3d& force_sp_N);

      Eigen::Matrix<double, 4, 4> build_physical_effectiveness_matrix() const;
      Eigen::Matrix<double, 4, 1> allocate_thrust(
        const Eigen::Matrix<double, 4, 4>& effectiveness,
        const Eigen::Matrix<double, 4, 1>& u_phys) const;
      Eigen::Matrix<double, 4, 1> thrust_to_motor_speed(
        const Eigen::Matrix<double, 4, 1>& thrust) const;
      Eigen::Matrix<double, 4, 1> thrust_to_normalized_motor_command(
        const Eigen::Matrix<double, 4, 1>& thrust) const;

      RateController rate_controller_;
      Eigen::Vector3d omega_sp_ = Eigen::Vector3d::Zero();
      Eigen::Vector3d omega_measured_ = Eigen::Vector3d::Zero();
      Eigen::Vector3d force_sp_ = Eigen::Vector3d::Zero();
      bool have_rate_setpoint_ = false;
      bool have_angular_velocity_ = false;
      bool have_force_setpoint_ = false;
      bool have_wrench_ = false;
      Eigen::Matrix<double, 4, 1> wrench_ = Eigen::Matrix<double, 4, 1>::Zero();

      double motor_min_ = -1.0;
      double motor_max_ = 1.0;

      Eigen::Matrix<double, 3, 4> quad_positions_;
      Eigen::Vector4d quad_sigmas_;
      double quad_moment_constant_ = 0.016;

      Eigen::Vector4d motor_constants_;
      Eigen::Vector4d max_rot_velocities_;
      Eigen::Vector4d thrust_trim_;

      rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rate_setpoint_sub_;
      rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr force_setpoint_sub_;

      rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr shuttle_motors_pub_;
      rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
      px4_msgs::msg::OffboardControlMode offboard_msg_;
      rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
      rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr mode_pub_;

      rclcpp::TimerBase::SharedPtr offboard_heartbeat_timer_;

      px4_msgs::msg::ActuatorMotors shuttle_motors_msg_;
  };
}
