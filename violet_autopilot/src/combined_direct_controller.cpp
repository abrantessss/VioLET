#include "combined_direct_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace autopilot {
  namespace {
    Eigen::Matrix<double, Eigen::Dynamic, 1> vector_parameter_or_default(
      const std::vector<double>& values,
      const Eigen::Matrix<double, Eigen::Dynamic, 1>& fallback,
      const rclcpp::Logger& logger,
      const std::string& name) {
      if (values.size() != static_cast<std::size_t>(fallback.size())) {
        RCLCPP_WARN(
          logger,
          "Parameter %s has %zu entries, expected %ld; using defaults",
          name.c_str(),
          values.size(),
          fallback.size());
        return fallback;
      }

      Eigen::Matrix<double, Eigen::Dynamic, 1> result(values.size());
      for (std::size_t i = 0; i < values.size(); ++i) {
        result(static_cast<Eigen::Index>(i)) = values[i];
      }
      return result;
    }
  }

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
    node_->declare_parameter<std::vector<double>>(
      "controllers.combineddirectcontroller.wrench_scales",
      {1.0, 1.0, 1.0, 1.0, 1.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combineddirectcontroller.quad_positions",
      {
        0.248, -0.248, -0.51,
       -0.248,  0.248, -0.51,
        0.248,  0.248, -0.51,
       -0.248, -0.248, -0.51
      });
    node_->declare_parameter<std::vector<double>>(
      "controllers.combineddirectcontroller.quad_sigmas",
      {1.0, 1.0, -1.0, -1.0});
    node_->declare_parameter<double>(
      "controllers.combineddirectcontroller.quad_moment_constant",
      0.016);
    node_->declare_parameter<std::vector<double>>(
      "controllers.combineddirectcontroller.fixed_wing_prop_position",
      {0.345, 0.0, 0.306});
    node_->declare_parameter<double>(
      "controllers.combineddirectcontroller.fixed_wing_sigma",
      -1.0);
    node_->declare_parameter<double>(
      "controllers.combineddirectcontroller.fixed_wing_moment_constant",
      0.01);
    node_->declare_parameter<std::vector<double>>(
      "controllers.combineddirectcontroller.motor_constants",
      {1.709716e-05, 1.709716e-05, 1.709716e-05, 1.709716e-05, 8.54858e-06});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combineddirectcontroller.max_rot_velocities",
      {1400.0, 1400.0, 1400.0, 1400.0, 3500.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combineddirectcontroller.thrust_trim",
      {0.0, 0.0, 0.0, 0.0, 0.0});

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
    motor_min_ =
      node_->get_parameter("controllers.combineddirectcontroller.limits.motor_min").as_double();
    motor_max_ =
      node_->get_parameter("controllers.combineddirectcontroller.limits.motor_max").as_double();
    servo_min_ =
      node_->get_parameter("controllers.combineddirectcontroller.limits.servo_min").as_double();
    servo_max_ =
      node_->get_parameter("controllers.combineddirectcontroller.limits.servo_max").as_double();
    quad_moment_constant_ =
      node_->get_parameter("controllers.combineddirectcontroller.quad_moment_constant").as_double();
    fixed_wing_sigma_ =
      node_->get_parameter("controllers.combineddirectcontroller.fixed_wing_sigma").as_double();
    fixed_wing_moment_constant_ =
      node_->get_parameter("controllers.combineddirectcontroller.fixed_wing_moment_constant").as_double();

    Eigen::Matrix<double, Eigen::Dynamic, 1> default_wrench_scale(5);
    default_wrench_scale << 1.0, 1.0, 1.0, 1.0, 1.0;
    Eigen::Matrix<double, Eigen::Dynamic, 1> default_quad_positions(12);
    default_quad_positions <<
       0.248, -0.248, -0.51,
      -0.248,  0.248, -0.51,
       0.248,  0.248, -0.51,
      -0.248, -0.248, -0.51;
    Eigen::Matrix<double, Eigen::Dynamic, 1> default_quad_sigmas(4);
    default_quad_sigmas << 1.0, 1.0, -1.0, -1.0;
    Eigen::Matrix<double, Eigen::Dynamic, 1> default_fixed_wing_position(3);
    default_fixed_wing_position << 0.345, 0.0, 0.306;
    Eigen::Matrix<double, Eigen::Dynamic, 1> default_motor_constants(5);
    default_motor_constants << 1.709716e-05, 1.709716e-05, 1.709716e-05, 1.709716e-05, 8.54858e-06;
    Eigen::Matrix<double, Eigen::Dynamic, 1> default_max_rot_velocities(5);
    default_max_rot_velocities << 1400.0, 1400.0, 1400.0, 1400.0, 3500.0;
    Eigen::Matrix<double, Eigen::Dynamic, 1> default_thrust_trim(5);
    default_thrust_trim << 0.0, 0.0, 0.0, 0.0, 0.0;

    wrench_scale_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combineddirectcontroller.wrench_scales").as_double_array(),
      default_wrench_scale,
      node_->get_logger(),
      "controllers.combineddirectcontroller.wrench_scales");
    const Eigen::Matrix<double, Eigen::Dynamic, 1> quad_position_values =
      vector_parameter_or_default(
        node_->get_parameter("controllers.combineddirectcontroller.quad_positions").as_double_array(),
        default_quad_positions,
        node_->get_logger(),
        "controllers.combineddirectcontroller.quad_positions");
    for (int i = 0; i < 4; ++i) {
      quad_positions_.col(i) = quad_position_values.segment<3>(3 * i);
    }
    quad_sigmas_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combineddirectcontroller.quad_sigmas").as_double_array(),
      default_quad_sigmas,
      node_->get_logger(),
      "controllers.combineddirectcontroller.quad_sigmas");
    fixed_wing_prop_position_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combineddirectcontroller.fixed_wing_prop_position").as_double_array(),
      default_fixed_wing_position,
      node_->get_logger(),
      "controllers.combineddirectcontroller.fixed_wing_prop_position");
    motor_constants_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combineddirectcontroller.motor_constants").as_double_array(),
      default_motor_constants,
      node_->get_logger(),
      "controllers.combineddirectcontroller.motor_constants");
    max_rot_velocities_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combineddirectcontroller.max_rot_velocities").as_double_array(),
      default_max_rot_velocities,
      node_->get_logger(),
      "controllers.combineddirectcontroller.max_rot_velocities");
    thrust_trim_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combineddirectcontroller.thrust_trim").as_double_array(),
      default_thrust_trim,
      node_->get_logger(),
      "controllers.combineddirectcontroller.thrust_trim");

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
    if (motor_min_ > motor_max_) {
      std::swap(motor_min_, motor_max_);
    }
    if (servo_min_ > servo_max_) {
      std::swap(servo_min_, servo_max_);
    }
    for (int i = 0; i < 5; ++i) {
      if (motor_constants_(i) <= 0.0) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Motor constant %d must be positive; using default %.8g",
          i,
          default_motor_constants(i));
        motor_constants_(i) = default_motor_constants(i);
      }
      if (max_rot_velocities_(i) <= 0.0) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Max rotational velocity %d must be positive; using default %.3f",
          i,
          default_max_rot_velocities(i));
        max_rot_velocities_(i) = default_max_rot_velocities(i);
      }
    }

    offboard_heartbeat_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&CombinedDirectController::publish_offboard_heartbeat, this));

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
  }

  void CombinedDirectController::on_wrench_callback(
    const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
    wrench_ <<
      msg->wrench.torque.x,
      msg->wrench.torque.y,
      msg->wrench.torque.z,
      msg->wrench.force.x,
      msg->wrench.force.z;
    have_wrench_ = true;

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "CombinedDirectController received wrench: M=[%.3f %.3f %.3f], Fx=%.3f, Fz=%.3f, ignored Fy=%.3f",
      wrench_(0),
      wrench_(1),
      wrench_(2),
      wrench_(3),
      wrench_(4),
      msg->wrench.force.y);
  }

  Eigen::Matrix<double, 5, 5> CombinedDirectController::build_physical_effectiveness_matrix() const {
    Eigen::Matrix<double, 5, 5> effectiveness = Eigen::Matrix<double, 5, 5>::Zero();

    for (int i = 0; i < 4; ++i) {
      effectiveness(0, i) = quad_positions_(1, i);
      effectiveness(1, i) = quad_positions_(0, i);
      effectiveness(2, i) = quad_sigmas_(i) * quad_moment_constant_;
      effectiveness(4, i) = -1.0;
    }

    effectiveness(0, 4) = fixed_wing_sigma_ * fixed_wing_moment_constant_;
    effectiveness(1, 4) = fixed_wing_prop_position_(2);
    effectiveness(2, 4) = -fixed_wing_prop_position_(1);
    effectiveness(3, 4) = 1.0;

    return effectiveness;
  }

  Eigen::Matrix<double, 5, 1> CombinedDirectController::normalized_wrench_to_physical(
    const Eigen::Matrix<double, 5, 1>& u_norm) const {
    return wrench_scale_.cwiseProduct(u_norm);
  }

  Eigen::Matrix<double, 5, 1> CombinedDirectController::allocate_thrust(
    const Eigen::Matrix<double, 5, 5>& effectiveness,
    const Eigen::Matrix<double, 5, 1>& u_phys) const {
    return effectiveness.inverse() * u_phys;
  }

  Eigen::Matrix<double, 5, 1> CombinedDirectController::thrust_to_motor_speed(
    const Eigen::Matrix<double, 5, 1>& thrust) const {
    Eigen::Matrix<double, 5, 1> omega;
    for (int i = 0; i < 5; ++i) {
      omega(i) = std::sqrt(std::max(0.0, thrust(i)) / motor_constants_(i));
    }
    return omega;
  }

  Eigen::Matrix<double, 5, 1> CombinedDirectController::thrust_to_normalized_motor_command(
    const Eigen::Matrix<double, 5, 1>& thrust) const {
    Eigen::Matrix<double, 5, 1> motor_command;
    for (int i = 0; i < 5; ++i) {
      const double thrust_max =
        motor_constants_(i) * max_rot_velocities_(i) * max_rot_velocities_(i);
      motor_command(i) = std::sqrt(std::clamp(thrust(i), 0.0, thrust_max) / thrust_max);
    }
    return motor_command;
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

    const Eigen::Matrix<double, 5, 5> effectiveness = build_physical_effectiveness_matrix();
    const Eigen::Matrix<double, 5, 1> u_norm = wrench_;
    const Eigen::Matrix<double, 5, 1> u_phys = normalized_wrench_to_physical(u_norm);
    const Eigen::Matrix<double, 5, 1> delta_thrust = allocate_thrust(effectiveness, u_phys);
    const Eigen::Matrix<double, 5, 1> thrust_before_saturation = thrust_trim_ + delta_thrust;
    Eigen::Matrix<double, 5, 1> thrust_after_saturation;
    bool clipped = false;
    for (int i = 0; i < 5; ++i) {
      const double thrust_max =
        motor_constants_(i) * max_rot_velocities_(i) * max_rot_velocities_(i);
      thrust_after_saturation(i) = std::clamp(thrust_before_saturation(i), 0.0, thrust_max);
      clipped = clipped || thrust_after_saturation(i) != thrust_before_saturation(i);
    }
    const Eigen::Matrix<double, 5, 1> omega_cmd = thrust_to_motor_speed(thrust_after_saturation);
    const Eigen::Matrix<double, 5, 1> actuator_commands =
      thrust_to_normalized_motor_command(thrust_after_saturation);
    const Eigen::Matrix<double, 5, 1> achieved_wrench =
      effectiveness * (thrust_after_saturation - thrust_trim_);
    const Eigen::Matrix<double, 5, 1> wrench_error = u_phys - achieved_wrench;

    const float nan = std::numeric_limits<float>::quiet_NaN();

    shuttle_motors_msg_.timestamp = now_us;
    shuttle_motors_msg_.timestamp_sample = now_us;
    shuttle_motors_msg_.reversible_flags = 0;
    shuttle_motors_msg_.control.fill(nan);
    for (int i = 0; i < 4; ++i) {
      shuttle_motors_msg_.control[i] =
        static_cast<float>(std::clamp(actuator_commands(i), motor_min_, motor_max_));
    }

    shuttle_motors_pub_->publish(shuttle_motors_msg_);

    fixed_wing_motors_msg_.timestamp = now_us;
    fixed_wing_motors_msg_.timestamp_sample = now_us;
    fixed_wing_motors_msg_.reversible_flags = 0;
    fixed_wing_motors_msg_.control.fill(nan);
    fixed_wing_motors_msg_.control[0] =
      static_cast<float>(std::clamp(actuator_commands(4), 0.0, motor_max_));
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
      "CombinedDirectController physical allocation%s\n"
      "  axes=[Mx My Mz Fx Fz]\n"
      "  u_norm=[%.3f %.3f %.3f %.3f %.3f]\n"
      "  u_phys=[%.3f %.3f %.3f %.3f %.3f]\n"
      "  Delta_T=[%.3f %.3f %.3f %.3f %.3f]\n"
      "  T_before_sat=[%.3f %.3f %.3f %.3f %.3f]\n"
      "  T_after_sat=[%.3f %.3f %.3f %.3f %.3f]\n"
      "  omega_cmd=[%.3f %.3f %.3f %.3f %.3f]\n"
      "  motor_cmd=[%.3f %.3f %.3f %.3f %.3f]\n"
      "  u_achieved=[%.3f %.3f %.3f %.3f %.3f]\n"
      "  u_error=[%.3f %.3f %.3f %.3f %.3f]",
      clipped ? " clipped" : "",
      u_norm(0), u_norm(1), u_norm(2), u_norm(3), u_norm(4),
      u_phys(0), u_phys(1), u_phys(2), u_phys(3), u_phys(4),
      delta_thrust(0), delta_thrust(1), delta_thrust(2), delta_thrust(3), delta_thrust(4),
      thrust_before_saturation(0), thrust_before_saturation(1), thrust_before_saturation(2),
      thrust_before_saturation(3), thrust_before_saturation(4),
      thrust_after_saturation(0), thrust_after_saturation(1), thrust_after_saturation(2),
      thrust_after_saturation(3), thrust_after_saturation(4),
      omega_cmd(0), omega_cmd(1), omega_cmd(2), omega_cmd(3), omega_cmd(4),
      actuator_commands(0), actuator_commands(1), actuator_commands(2),
      actuator_commands(3), actuator_commands(4),
      achieved_wrench(0), achieved_wrench(1), achieved_wrench(2),
      achieved_wrench(3), achieved_wrench(4),
      wrench_error(0), wrench_error(1), wrench_error(2),
      wrench_error(3), wrench_error(4));

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "CombinedDirectController actuator commands: q=[%.3f %.3f %.3f %.3f], prop=%.3f, surf=[%.3f %.3f %.3f %.3f]",
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

  void CombinedDirectController::publish_offboard_heartbeat() {
    publish_offboard_mode();
    publish_offboard_request(shuttle_mode_pub_, vehicle_id_);
    publish_offboard_request(fixed_wing_mode_pub_, fixed_wing_vehicle_id_);
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
