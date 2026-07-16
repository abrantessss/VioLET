#include "shuttle_rate_controller.hpp"

#include <chrono>
#include <functional>
#include <string>

namespace autopilot {

  ShuttleRateController::~ShuttleRateController() {}

  void ShuttleRateController::initialize() {
    node_->declare_parameter<std::string>(
      "controllers.shuttleratecontroller.subscribers.rate_setpoint",
      "controller/in/rate_setpoint");
    node_->declare_parameter<std::string>(
      "controllers.shuttleratecontroller.publishers.setpoint",
      "fmu/in/vehicle_rates_setpoint");
    node_->declare_parameter<double>(
      "controllers.shuttleratecontroller.publish_rate_hz",
      50.0);

    rate_setpoint_sub_ = node_->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
      node_->get_parameter("controllers.shuttleratecontroller.subscribers.rate_setpoint").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&ShuttleRateController::on_rate_setpoint_callback, this, std::placeholders::_1));

    rates_pub_ = node_->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
      node_->get_parameter("controllers.shuttleratecontroller.publishers.setpoint").as_string(),
      rclcpp::SensorDataQoS());

    node_->declare_parameter<std::string>("publishers.mode.offboard", "fmu/in/offboard_control_mode");
    offboard_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
      node_->get_parameter("publishers.mode.offboard").as_string(),
      rclcpp::SensorDataQoS());

    node_->declare_parameter<std::string>("publishers.mode.request", "fmu/in/vehicle_command");
    mode_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
      node_->get_parameter("publishers.mode.request").as_string(),
      rclcpp::SensorDataQoS());

    publish_rate_hz_ =
      node_->get_parameter("controllers.shuttleratecontroller.publish_rate_hz").as_double();
    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 50.0;
    }

    setpoint_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&ShuttleRateController::publish_rate_setpoint, this));

    RCLCPP_INFO(
      node_->get_logger(),
      "ShuttleRateController initialized at %.1f Hz",
      publish_rate_hz_);
  }

  void ShuttleRateController::reset() {
    have_rate_setpoint_ = false;
    rates_msg_ = px4_msgs::msg::VehicleRatesSetpoint();
  }

  void ShuttleRateController::on_rate_setpoint_callback(
    const px4_msgs::msg::VehicleRatesSetpoint::ConstSharedPtr msg) {
    rates_msg_ = *msg;
    have_rate_setpoint_ = true;
  }

  void ShuttleRateController::set_position(const double dt, const Eigen::Vector3d& p) {
    (void)dt;
    (void)p;
  }

  void ShuttleRateController::set_attitude(
    const double dt,
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& v) {
    (void)dt;
    (void)p;
    (void)v;
  }

  void ShuttleRateController::set_attitude_rate(
    const double dt,
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& v,
    const Eigen::Vector3d& eta) {
    (void)dt;
    (void)p;
    (void)v;
    (void)eta;
  }

  void ShuttleRateController::set_path(const int type, const double* path) {
    (void)type;
    (void)path;
    publish_offboard_request();
  }

  void ShuttleRateController::publish_rate_setpoint() {
    publish_offboard_mode();

    if (!have_rate_setpoint_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "ShuttleRateController waiting for rate setpoint");
      return;
    }

    rates_msg_.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    rates_pub_->publish(rates_msg_);
  }

  void ShuttleRateController::publish_offboard_mode() {
    offboard_msg_.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    offboard_msg_.position = false;
    offboard_msg_.velocity = false;
    offboard_msg_.acceleration = false;
    offboard_msg_.attitude = false;
    offboard_msg_.body_rate = true;
    offboard_msg_.thrust_and_torque = false;
    offboard_msg_.direct_actuator = false;

    offboard_pub_->publish(offboard_msg_);
  }

  void ShuttleRateController::publish_offboard_request() {
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
