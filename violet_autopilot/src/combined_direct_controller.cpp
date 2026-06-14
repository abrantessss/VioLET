#include "combined_direct_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace autopilot {

  CombinedDirectController::~CombinedDirectController() {}

  void CombinedDirectController::initialize() {
    node_->declare_parameter<std::string>(
      "controllers.combineddirectcontroller.subscribers.wrench",
      "fmu/in/combined_wrench");
    node_->declare_parameter<std::string>(
      "controllers.combineddirectcontroller.publishers.shuttle_motors",
      "fmu/in/actuator_motors");
    node_->declare_parameter<std::string>(
      "controllers.combineddirectcontroller.fixed_wing_namespace",
      "drone2");
    node_->declare_parameter<int>(
      "controllers.combineddirectcontroller.fixed_wing_vehicle_id",
      2);
    node_->declare_parameter<double>(
      "controllers.combineddirectcontroller.publish_rate_hz",
      50.0);
    node_->declare_parameter<double>(
      "controllers.combineddirectcontroller.damping",
      0.01);
    node_->declare_parameter<double>(
      "controllers.combineddirectcontroller.limits.motor_min",
      -1.0);
    node_->declare_parameter<double>(
      "controllers.combineddirectcontroller.limits.motor_max",
      1.0);
    node_->declare_parameter<double>(
      "controllers.combineddirectcontroller.limits.servo_min",
      -1.0);
    node_->declare_parameter<double>(
      "controllers.combineddirectcontroller.limits.servo_max",
      1.0);

    const std::string wrench_topic =
      node_->get_parameter("controllers.combineddirectcontroller.subscribers.wrench").as_string();
    const std::string shuttle_motors_topic =
      node_->get_parameter("controllers.combineddirectcontroller.publishers.shuttle_motors").as_string();

    fixed_wing_namespace_ =
      node_->get_parameter("controllers.combineddirectcontroller.fixed_wing_namespace").as_string();
    fixed_wing_vehicle_id_ =
      node_->get_parameter("controllers.combineddirectcontroller.fixed_wing_vehicle_id").as_int();
    publish_rate_hz_ =
      node_->get_parameter("controllers.combineddirectcontroller.publish_rate_hz").as_double();
    damping_ =
      node_->get_parameter("controllers.combineddirectcontroller.damping").as_double();
    motor_min_ =
      node_->get_parameter("controllers.combineddirectcontroller.limits.motor_min").as_double();
    motor_max_ =
      node_->get_parameter("controllers.combineddirectcontroller.limits.motor_max").as_double();
    servo_min_ =
      node_->get_parameter("controllers.combineddirectcontroller.limits.servo_min").as_double();
    servo_max_ =
      node_->get_parameter("controllers.combineddirectcontroller.limits.servo_max").as_double();

    fixed_wing_namespace_.erase(0, fixed_wing_namespace_.find_first_not_of('/'));
    fixed_wing_namespace_.erase(fixed_wing_namespace_.find_last_not_of('/') + 1);
    const std::string fixed_wing_prefix = "/" + fixed_wing_namespace_;

    wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      wrench_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&CombinedDirectController::on_wrench_callback, this, std::placeholders::_1));

    shuttle_motors_pub_ = node_->create_publisher<px4_msgs::msg::ActuatorMotors>(
      shuttle_motors_topic,
      rclcpp::SensorDataQoS());
    fixed_wing_motors_pub_ = node_->create_publisher<px4_msgs::msg::ActuatorMotors>(
      fixed_wing_prefix + "/fmu/in/actuator_motors",
      rclcpp::SensorDataQoS());
    fixed_wing_servos_pub_ = node_->create_publisher<px4_msgs::msg::ActuatorServos>(
      fixed_wing_prefix + "/fmu/in/actuator_servos",
      rclcpp::SensorDataQoS());

    node_->declare_parameter<std::string>("publishers.mode.offboard", "fmu/in/offboard_control_mode");
    node_->declare_parameter<std::string>("publishers.mode.request", "fmu/in/vehicle_command");
    shuttle_offboard_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
      node_->get_parameter("publishers.mode.offboard").as_string(),
      rclcpp::SensorDataQoS());
    shuttle_mode_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
      node_->get_parameter("publishers.mode.request").as_string(),
      rclcpp::SensorDataQoS());
    fixed_wing_offboard_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
      fixed_wing_prefix + "/fmu/in/offboard_control_mode",
      rclcpp::SensorDataQoS());
    fixed_wing_mode_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
      fixed_wing_prefix + "/fmu/in/vehicle_command",
      rclcpp::SensorDataQoS());

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 50.0;
    }
    if (damping_ <= 0.0) {
      damping_ = 0.01;
    }
    if (motor_min_ > motor_max_) {
      std::swap(motor_min_, motor_max_);
    }
    if (servo_min_ > servo_max_) {
      std::swap(servo_min_, servo_max_);
    }

    mixer_ <<
      -1.0,  1.0,  1.0, -1.0,  0.1,
       1.0, -1.0,  1.0, -1.0,  4.0,
       1.0,  1.0, -1.0, -1.0,  0.0,
       0.0,  0.0,  0.0,  0.0,  3.43,
       0.0,  0.0,  0.0,  0.0,  0.0,
      -1.0, -1.0, -1.0, -1.0,  0.0;

    RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "CombinedDirectController initialized. Wrench topic: "
        << wrench_topic << ", fixed-wing namespace: /"
        << fixed_wing_namespace_ << ", publish_rate_hz: "
        << publish_rate_hz_);
  }

  void CombinedDirectController::set_position(const double dt, const Eigen::Vector3d& p) {
    (void)dt;
    (void)p;
    publish_actuators();
  }

  void CombinedDirectController::set_attitude(
    const double dt,
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& v) {
    (void)dt;
    (void)p;
    (void)v;
    publish_actuators();
  }

  void CombinedDirectController::set_attitude_rate(
    const double dt,
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& v,
    const Eigen::Vector3d& eta) {
    (void)dt;
    (void)p;
    (void)v;
    (void)eta;
    publish_actuators();
  }

  void CombinedDirectController::set_path(const int type, const double* path) {
    (void)type;
    (void)path;
    request_offboard_until_us_ = node_->get_clock()->now().nanoseconds() / 1000 + 3000000;
  }

  void CombinedDirectController::on_wrench_callback(
    const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
    wrench_ <<
      msg->wrench.torque.x,
      msg->wrench.torque.y,
      msg->wrench.torque.z,
      msg->wrench.force.x,
      msg->wrench.force.y,
      msg->wrench.force.z;
    have_wrench_ = true;

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "CombinedDirectController received wrench: M=[%.3f %.3f %.3f], F=[%.3f %.3f %.3f]",
      wrench_(0),
      wrench_(1),
      wrench_(2),
      wrench_(3),
      wrench_(4),
      wrench_(5));
  }

  Eigen::Matrix<double, 5, 1> CombinedDirectController::mix_wrench() const {
    const Eigen::Matrix<double, 6, 6> regularized =
      mixer_ * mixer_.transpose()
      + damping_ * damping_ * Eigen::Matrix<double, 6, 6>::Identity();
    return mixer_.transpose() * regularized.ldlt().solve(wrench_);
  }

  void CombinedDirectController::publish_actuators() {
    if (!have_wrench_) {
      return;
    }

    const uint64_t now_us = node_->get_clock()->now().nanoseconds() / 1000;
    const uint64_t publish_period_us = static_cast<uint64_t>(1000000.0 / publish_rate_hz_);
    if (last_publish_us_ != 0 && now_us - last_publish_us_ < publish_period_us) {
      return;
    }
    last_publish_us_ = now_us;

    const Eigen::Matrix<double, 5, 1> mixed = mix_wrench();
    const float nan = std::numeric_limits<float>::quiet_NaN();

    shuttle_motors_msg_.timestamp = now_us;
    shuttle_motors_msg_.timestamp_sample = now_us;
    shuttle_motors_msg_.reversible_flags = 0;
    shuttle_motors_msg_.control.fill(nan);
    for (int i = 0; i < 4; ++i) {
      shuttle_motors_msg_.control[i] =
        static_cast<float>(std::clamp(mixed(i), motor_min_, motor_max_));
    }
    publish_offboard_mode();

    if (now_us < request_offboard_until_us_) {
      publish_offboard_request(shuttle_mode_pub_, vehicle_id_);
      publish_offboard_request(fixed_wing_mode_pub_, fixed_wing_vehicle_id_);
    }

    shuttle_motors_pub_->publish(shuttle_motors_msg_);

    fixed_wing_motors_msg_.timestamp = now_us;
    fixed_wing_motors_msg_.timestamp_sample = now_us;
    fixed_wing_motors_msg_.reversible_flags = 0;
    fixed_wing_motors_msg_.control.fill(nan);
    fixed_wing_motors_msg_.control[0] =
      static_cast<float>(std::clamp(mixed(4), 0.0, motor_max_));
    fixed_wing_motors_pub_->publish(fixed_wing_motors_msg_);

    fixed_wing_servos_msg_.timestamp = now_us;
    fixed_wing_servos_msg_.timestamp_sample = now_us;
    fixed_wing_servos_msg_.control.fill(nan);
    fixed_wing_servos_msg_.control[0] = 0.0F;
    fixed_wing_servos_msg_.control[1] = 0.0F;
    fixed_wing_servos_msg_.control[2] = 0.0F;
    fixed_wing_servos_msg_.control[3] = 0.0F;
    fixed_wing_servos_pub_->publish(fixed_wing_servos_msg_);

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "CombinedDirectController actuator output: q=[%.3f %.3f %.3f %.3f], prop=%.3f, surf=[%.3f %.3f %.3f %.3f]",
      shuttle_motors_msg_.control[0],
      shuttle_motors_msg_.control[1],
      shuttle_motors_msg_.control[2],
      shuttle_motors_msg_.control[3],
      fixed_wing_motors_msg_.control[0],
      fixed_wing_servos_msg_.control[0],
      fixed_wing_servos_msg_.control[1],
      fixed_wing_servos_msg_.control[2],
      fixed_wing_servos_msg_.control[3]);
  }

  void CombinedDirectController::publish_offboard_mode() {
    offboard_msg_.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    offboard_msg_.position = false;
    offboard_msg_.velocity = false;
    offboard_msg_.acceleration = false;
    offboard_msg_.attitude = false;
    offboard_msg_.body_rate = false;
    offboard_msg_.thrust_and_torque = false;
    offboard_msg_.direct_actuator = true;

    shuttle_offboard_pub_->publish(offboard_msg_);
    fixed_wing_offboard_pub_->publish(offboard_msg_);
  }

  void CombinedDirectController::publish_offboard_request(
    const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr& publisher,
    const int target_system) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    msg.param1 = 1;
    msg.param2 = 6;
    msg.target_system = target_system;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    publisher->publish(msg);
  }
}
