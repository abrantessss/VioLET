#pragma once

#include <Eigen/Dense>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/actuator_servos.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "violet_msgs/msg/state.hpp"

#include <controller.hpp>
#include <rate_controller.hpp>

namespace autopilot {
  class CombinedAttitudeController : public autopilot::Controller {
    public:
      ~CombinedAttitudeController();

      void initialize() override;

      void set_position(const double dt, const Eigen::Vector3d& p) override;

      void set_attitude(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v) override;

      void set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) override;

      void set_path(const int type, const double* path) override;
      void reset() override;

    protected:
      void on_wrench_callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg);
      void on_attitude_setpoint_callback(
        const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg);
      void on_state_callback(const violet_msgs::msg::State::ConstSharedPtr msg);
      void on_force_setpoint_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg);
      void update_rate_controller(double dt);
      void publish_wrench(const Eigen::Vector3d& torque_sp_Nm, const Eigen::Vector3d& force_sp_N);
      void publish_actuators();
      void publish_offboard_heartbeat();
      void publish_offboard_mode();
      void publish_offboard_request(
        const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr& publisher,
        const int target_system);
      Eigen::Matrix<double, 5, 5> build_physical_effectiveness_matrix() const;
      Eigen::Matrix<double, 5, 1> normalized_wrench_to_physical(
        const Eigen::Matrix<double, 5, 1>& u_command) const;
      Eigen::Matrix<double, 5, 1> allocate_thrust(
        const Eigen::Matrix<double, 5, 5>& effectiveness,
        const Eigen::Matrix<double, 5, 1>& u_phys) const;
      Eigen::Matrix<double, 5, 1> thrust_to_motor_speed(
        const Eigen::Matrix<double, 5, 1>& thrust) const;
      Eigen::Matrix<double, 5, 1> thrust_to_normalized_motor_command(
        const Eigen::Matrix<double, 5, 1>& thrust) const;

      Eigen::Matrix<double, 5, 1> wrench_{Eigen::Matrix<double, 5, 1>::Zero()};
      Eigen::Matrix<double, 5, 1> wrench_scale_{Eigen::Matrix<double, 5, 1>::Ones()};
      Eigen::Matrix<double, 3, 4> quad_positions_;
      Eigen::Matrix<double, 4, 1> quad_sigmas_;
      Eigen::Matrix<double, 3, 1> fixed_wing_prop_position_;
      Eigen::Matrix<double, 5, 1> motor_constants_;
      Eigen::Matrix<double, 5, 1> max_rot_velocities_;
      Eigen::Matrix<double, 5, 1> thrust_trim_;
      double quad_moment_constant_{0.016};
      double fixed_wing_moment_constant_{0.126};
      double fixed_wing_sigma_{1.0};

      px4_msgs::msg::ActuatorMotors shuttle_motors_msg_;
      px4_msgs::msg::ActuatorMotors fixed_wing_motors_msg_;
      px4_msgs::msg::ActuatorServos fixed_wing_servos_msg_;
      px4_msgs::msg::OffboardControlMode offboard_msg_;

      rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr shuttle_motors_pub_;
      rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr fixed_wing_motors_pub_;
      rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr fixed_wing_servos_pub_;
      rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr shuttle_offboard_pub_;
      rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr fixed_wing_offboard_pub_;
      rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr shuttle_mode_pub_;
      rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr fixed_wing_mode_pub_;
      rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
      rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
      rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_setpoint_sub_;
      rclcpp::Subscription<violet_msgs::msg::State>::SharedPtr state_sub_;
      rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr force_setpoint_sub_;
      rclcpp::TimerBase::SharedPtr offboard_heartbeat_timer_;

      std::string fixed_wing_namespace_{"drone2"};
      int fixed_wing_vehicle_id_{2};
      double motor_min_{-1.0};
      double motor_max_{1.0};
      double servo_min_{-1.0};
      double servo_max_{1.0};
      bool have_wrench_{false};
      RateController rate_controller_;
      Eigen::Vector3d attitude_kp_{Eigen::Vector3d::Ones()};
      Eigen::Vector3d attitude_rate_limits_{Eigen::Vector3d::Ones()};
      Eigen::Vector3d attitude_sp_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d previous_attitude_sp_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d attitude_measured_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d omega_sp_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d omega_measured_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d force_sp_{Eigen::Vector3d::Zero()};
      bool have_rate_setpoint_{false};
      bool have_attitude_setpoint_{false};
      bool have_previous_attitude_setpoint_{false};
      bool have_attitude_{false};
      bool have_angular_velocity_{false};
      bool have_force_setpoint_{false};
  };
}
