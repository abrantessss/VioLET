#pragma once

#include <Eigen/Dense>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/actuator_servos.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

#include <controller.hpp>

namespace autopilot {
  class CombinedDirectController : public autopilot::Controller {
    public:
      ~CombinedDirectController();

      void initialize() override;

      void set_position(const double dt, const Eigen::Vector3d& p) override;

      void set_attitude(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v) override;

      void set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) override;

      void set_path(const int type, const double* path) override;

    protected:
      void on_wrench_callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg);
      void publish_actuators();
      void publish_offboard_mode();
      void publish_offboard_request(
        const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr& publisher,
        const int target_system);
      Eigen::Matrix<double, 5, 1> mix_wrench() const;

      Eigen::Matrix<double, 6, 5> mixer_;
      Eigen::Matrix<double, 6, 1> wrench_{Eigen::Matrix<double, 6, 1>::Zero()};

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
      rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

      std::string fixed_wing_namespace_{"drone2"};
      int fixed_wing_vehicle_id_{2};
      double publish_rate_hz_{50.0};
      double damping_{0.01};
      double motor_min_{-1.0};
      double motor_max_{1.0};
      double servo_min_{-1.0};
      double servo_max_{1.0};
      uint64_t last_publish_us_{0};
      uint64_t request_offboard_until_us_{0};
      bool have_wrench_{false};
  };
}
