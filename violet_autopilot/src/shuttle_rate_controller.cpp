#include "shuttle_rate_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
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

    RateController::AxisLimit limit_parameter_or_default(
      const std::vector<double>& values,
      const RateController::AxisLimit& fallback,
      const rclcpp::Logger& logger,
      const std::string& name) {
      if (values.size() != 2U) {
        RCLCPP_WARN(
          logger,
          "Parameter %s has %zu entries, expected 2; using defaults",
          name.c_str(),
          values.size());
        return fallback;
      }

      RateController::AxisLimit result{values[0], values[1]};
      if (result.min > result.max) {
        RCLCPP_WARN(logger, "Parameter %s has min > max; swapping values", name.c_str());
        std::swap(result.min, result.max);
      }
      return result;
    }

    template <typename Derived>
    std::string vector_to_string(const Eigen::MatrixBase<Derived>& values) {
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(3) << "[";
      for (Eigen::Index i = 0; i < values.size(); ++i) {
        if (i != 0) {
          stream << " ";
        }
        stream << values(i);
      }
      stream << "]";
      return stream.str();
    }

    std::string actuator_allocation_table(
      const Eigen::Matrix<double, 4, 1>& thrust_before_saturation,
      const Eigen::Matrix<double, 4, 1>& thrust_after_saturation,
      const Eigen::Matrix<double, 4, 1>& omega_cmd,
      const Eigen::Matrix<double, 4, 1>& actuator_commands) {
      static constexpr const char* labels[4] = {"q1", "q2", "q3", "q4"};

      std::ostringstream stream;
      stream << std::fixed << std::setprecision(3);
      stream << "  actuator   T_req_N   T_cmd_N   clipped_N   omega_rad_s   cmd\n";
      for (int i = 0; i < 4; ++i) {
        stream << "  " << std::setw(8) << labels[i] << std::setw(10)
               << thrust_before_saturation(i) << std::setw(10) << thrust_after_saturation(i)
               << std::setw(12) << thrust_after_saturation(i) - thrust_before_saturation(i)
               << std::setw(14) << omega_cmd(i) << std::setw(8) << actuator_commands(i) << "\n";
      }
      return stream.str();
    }

    bool contains_negative_thrust(const Eigen::Matrix<double, 4, 1>& thrust) {
      for (Eigen::Index i = 0; i < thrust.size(); ++i) {
        if (thrust(i) < 0.0) {
          return true;
        }
      }
      return false;
    }
  }  // namespace

  ShuttleRateController::~ShuttleRateController() {}

  void ShuttleRateController::initialize() {
    node_->declare_parameter<std::string>(
      "controllers.shuttleratecontroller.publishers.shuttle_motors", "fmu/in/actuator_motors");
    node_->declare_parameter<double>("controllers.shuttleratecontroller.limits.motor_min", -1.0);
    node_->declare_parameter<double>("controllers.shuttleratecontroller.limits.motor_max", 1.0);
    node_->declare_parameter<std::vector<double>>(
      "controllers.shuttleratecontroller.quad_positions",
      {0.248, -0.248, -0.51, -0.248, 0.248, -0.51, 0.248, 0.248, -0.51, -0.248, -0.248, -0.51});
    node_->declare_parameter<std::vector<double>>(
      "controllers.shuttleratecontroller.quad_sigmas", {1.0, 1.0, -1.0, -1.0});
    node_->declare_parameter<double>("controllers.shuttleratecontroller.quad_moment_constant", 0.016);
    node_->declare_parameter<std::vector<double>>(
      "controllers.shuttleratecontroller.motor_constants",
      {1.709716e-05, 1.709716e-05, 1.709716e-05, 1.709716e-05});
    node_->declare_parameter<std::vector<double>>(
      "controllers.shuttleratecontroller.max_rot_velocities", {1400.0, 1400.0, 1400.0, 1400.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.shuttleratecontroller.thrust_trim", {0.0, 0.0, 0.0, 0.0});
    node_->declare_parameter<double>("controllers.shuttleratecontroller.gains.roll.kp", 0.25);
    node_->declare_parameter<double>("controllers.shuttleratecontroller.gains.roll.ki", 0.0);
    node_->declare_parameter<double>("controllers.shuttleratecontroller.gains.roll.kd", 0.0);
    node_->declare_parameter<double>("controllers.shuttleratecontroller.gains.pitch.kp", 0.25);
    node_->declare_parameter<double>("controllers.shuttleratecontroller.gains.pitch.ki", 0.0);
    node_->declare_parameter<double>("controllers.shuttleratecontroller.gains.pitch.kd", 0.0);
    node_->declare_parameter<double>("controllers.shuttleratecontroller.gains.yaw.kp", 0.10);
    node_->declare_parameter<double>("controllers.shuttleratecontroller.gains.yaw.ki", 0.0);
    node_->declare_parameter<double>("controllers.shuttleratecontroller.gains.yaw.kd", 0.0);
    node_->declare_parameter<std::string>("controllers.shuttleratecontroller.gain_form", "parallel");
    node_->declare_parameter<std::vector<double>>(
      "controllers.shuttleratecontroller.output_limits.roll", {-2.0, 2.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.shuttleratecontroller.output_limits.pitch", {-2.0, 2.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.shuttleratecontroller.output_limits.yaw", {-1.0, 1.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.shuttleratecontroller.integral_limits.roll", {-5.0, 5.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.shuttleratecontroller.integral_limits.pitch", {-5.0, 5.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.shuttleratecontroller.integral_limits.yaw", {-5.0, 5.0});
    node_->declare_parameter<std::string>(
      "controllers.shuttleratecontroller.subscribers.rate_setpoint", "controller/in/rate_setpoint");
    node_->declare_parameter<std::string>(
      "controllers.shuttleratecontroller.subscribers.force_setpoint", "controller/in/force_setpoint");
    node_->declare_parameter<std::string>(
      "controllers.shuttleratecontroller.publishers.wrench",
      "controller/out/wrench");

    const std::string shuttle_motors_topic =
      node_->get_parameter("controllers.shuttleratecontroller.publishers.shuttle_motors").as_string();
    const std::string rate_setpoint_topic =
      node_->get_parameter("controllers.shuttleratecontroller.subscribers.rate_setpoint").as_string();
    const std::string force_setpoint_topic =
      node_->get_parameter("controllers.shuttleratecontroller.subscribers.force_setpoint").as_string();
    const std::string wrench_topic =
      node_->get_parameter("controllers.shuttleratecontroller.publishers.wrench").as_string();

    motor_min_ = node_->get_parameter("controllers.shuttleratecontroller.limits.motor_min").as_double();
    motor_max_ = node_->get_parameter("controllers.shuttleratecontroller.limits.motor_max").as_double();
    quad_moment_constant_ =
      node_->get_parameter("controllers.shuttleratecontroller.quad_moment_constant").as_double();

    RateController::Config rate_config;
    rate_config.roll.kp =
      node_->get_parameter("controllers.shuttleratecontroller.gains.roll.kp").as_double();
    rate_config.roll.ki =
      node_->get_parameter("controllers.shuttleratecontroller.gains.roll.ki").as_double();
    rate_config.roll.kd =
      node_->get_parameter("controllers.shuttleratecontroller.gains.roll.kd").as_double();
    rate_config.pitch.kp =
      node_->get_parameter("controllers.shuttleratecontroller.gains.pitch.kp").as_double();
    rate_config.pitch.ki =
      node_->get_parameter("controllers.shuttleratecontroller.gains.pitch.ki").as_double();
    rate_config.pitch.kd =
      node_->get_parameter("controllers.shuttleratecontroller.gains.pitch.kd").as_double();
    rate_config.yaw.kp =
      node_->get_parameter("controllers.shuttleratecontroller.gains.yaw.kp").as_double();
    rate_config.yaw.ki =
      node_->get_parameter("controllers.shuttleratecontroller.gains.yaw.ki").as_double();
    rate_config.yaw.kd =
      node_->get_parameter("controllers.shuttleratecontroller.gains.yaw.kd").as_double();
    bool valid_gain_form = false;
    rate_config.gain_form = RateController::gain_form_from_string(
      node_->get_parameter("controllers.shuttleratecontroller.gain_form").as_string(),
      &valid_gain_form);
    if (!valid_gain_form) {
      RCLCPP_WARN(
        node_->get_logger(), "Invalid controllers.shuttleratecontroller.gain_form; using parallel");
    }
    rate_config.output_limits[0] = limit_parameter_or_default(
      node_->get_parameter("controllers.shuttleratecontroller.output_limits.roll").as_double_array(),
      {-2.0, 2.0},
      node_->get_logger(),
      "controllers.shuttleratecontroller.output_limits.roll");
    rate_config.output_limits[1] = limit_parameter_or_default(
      node_->get_parameter("controllers.shuttleratecontroller.output_limits.pitch").as_double_array(),
      {-2.0, 2.0},
      node_->get_logger(),
      "controllers.shuttleratecontroller.output_limits.pitch");
    rate_config.output_limits[2] = limit_parameter_or_default(
      node_->get_parameter("controllers.shuttleratecontroller.output_limits.yaw").as_double_array(),
      {-1.0, 1.0},
      node_->get_logger(),
      "controllers.shuttleratecontroller.output_limits.yaw");
    rate_config.integral_limits[0] = limit_parameter_or_default(
      node_->get_parameter("controllers.shuttleratecontroller.integral_limits.roll").as_double_array(),
      {-5.0, 5.0},
      node_->get_logger(),
      "controllers.shuttleratecontroller.integral_limits.roll");
    rate_config.integral_limits[1] = limit_parameter_or_default(
      node_->get_parameter("controllers.shuttleratecontroller.integral_limits.pitch")
        .as_double_array(),
      {-5.0, 5.0},
      node_->get_logger(),
      "controllers.shuttleratecontroller.integral_limits.pitch");
    rate_config.integral_limits[2] = limit_parameter_or_default(
      node_->get_parameter("controllers.shuttleratecontroller.integral_limits.yaw").as_double_array(),
      {-5.0, 5.0},
      node_->get_logger(),
      "controllers.shuttleratecontroller.integral_limits.yaw");
    rate_controller_.configure(rate_config);

    Eigen::Matrix<double, Eigen::Dynamic, 1> default_quad_positions(12);
    default_quad_positions << 0.248, -0.248, -0.51, -0.248, 0.248, -0.51, 0.248, 0.248, -0.51,
      -0.248, -0.248, -0.51;
    Eigen::Matrix<double, Eigen::Dynamic, 1> default_quad_sigmas(4);
    default_quad_sigmas << 1.0, 1.0, -1.0, -1.0;
    Eigen::Matrix<double, Eigen::Dynamic, 1> default_motor_constants(4);
    default_motor_constants << 1.709716e-05, 1.709716e-05, 1.709716e-05, 1.709716e-05;
    Eigen::Matrix<double, Eigen::Dynamic, 1> default_max_rot_velocities(4);
    default_max_rot_velocities << 1400.0, 1400.0, 1400.0, 1400.0;
    Eigen::Matrix<double, Eigen::Dynamic, 1> default_thrust_trim(4);
    default_thrust_trim << 0.0, 0.0, 0.0, 0.0;

    const Eigen::Matrix<double, Eigen::Dynamic, 1> quad_position_values =
      vector_parameter_or_default(
        node_->get_parameter("controllers.shuttleratecontroller.quad_positions").as_double_array(),
        default_quad_positions,
        node_->get_logger(),
        "controllers.shuttleratecontroller.quad_positions");
    for (int i = 0; i < 4; ++i) {
      quad_positions_.col(i) = quad_position_values.segment<3>(3 * i);
    }
    quad_sigmas_ = vector_parameter_or_default(
      node_->get_parameter("controllers.shuttleratecontroller.quad_sigmas").as_double_array(),
      default_quad_sigmas,
      node_->get_logger(),
      "controllers.shuttleratecontroller.quad_sigmas");
    motor_constants_ = vector_parameter_or_default(
      node_->get_parameter("controllers.shuttleratecontroller.motor_constants").as_double_array(),
      default_motor_constants,
      node_->get_logger(),
      "controllers.shuttleratecontroller.motor_constants");
    max_rot_velocities_ = vector_parameter_or_default(
      node_->get_parameter("controllers.shuttleratecontroller.max_rot_velocities").as_double_array(),
      default_max_rot_velocities,
      node_->get_logger(),
      "controllers.shuttleratecontroller.max_rot_velocities");
    thrust_trim_ = vector_parameter_or_default(
      node_->get_parameter("controllers.shuttleratecontroller.thrust_trim").as_double_array(),
      default_thrust_trim,
      node_->get_logger(),
      "controllers.shuttleratecontroller.thrust_trim");

    rate_setpoint_sub_ = node_->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
      rate_setpoint_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&ShuttleRateController::on_rate_setpoint_callback, this, std::placeholders::_1));
    force_setpoint_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      force_setpoint_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&ShuttleRateController::on_force_setpoint_callback, this, std::placeholders::_1));

    shuttle_motors_pub_ = node_->create_publisher<px4_msgs::msg::ActuatorMotors>(
      shuttle_motors_topic, rclcpp::SensorDataQoS());
    wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      wrench_topic, rclcpp::SensorDataQoS());

    node_->declare_parameter<std::string>("publishers.mode.offboard", "fmu/in/offboard_control_mode");
    node_->declare_parameter<std::string>("publishers.mode.request", "fmu/in/vehicle_command");
    offboard_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
      node_->get_parameter("publishers.mode.offboard").as_string(), rclcpp::SensorDataQoS());
    mode_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
      node_->get_parameter("publishers.mode.request").as_string(), rclcpp::SensorDataQoS());

    if (motor_min_ > motor_max_) {
      std::swap(motor_min_, motor_max_);
    }
    for (int i = 0; i < 4; ++i) {
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
      std::chrono::milliseconds(20), std::bind(&ShuttleRateController::publish_offboard_heartbeat, this));

    RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "ShuttleRateController initialized. Rate setpoint topic: "
        << rate_setpoint_topic << ", force setpoint topic: " << force_setpoint_topic);
  }

  void ShuttleRateController::set_position(const double dt, const Eigen::Vector3d& p) {
    (void)dt;
    (void)p;
    publish_actuators();
  }

  void ShuttleRateController::set_attitude(
    const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v) {
    (void)dt;
    (void)p;
    (void)v;
    publish_actuators();
  }

  void ShuttleRateController::set_attitude_rate(
    const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) {
    (void)p;
    (void)v;
    omega_measured_ = eta;
    have_angular_velocity_ = true;
    update_rate_controller(dt);
    publish_actuators();
  }

  void ShuttleRateController::set_path(const int type, const double* path) {
    (void)type;
    (void)path;
  }

  void ShuttleRateController::reset() {
    rate_controller_.reset();
    omega_sp_.setZero();
    omega_measured_.setZero();
    force_sp_.setZero();
    have_rate_setpoint_ = false;
    have_angular_velocity_ = false;
    have_force_setpoint_ = false;
    have_wrench_ = false;
  }

  void ShuttleRateController::on_rate_setpoint_callback(
    const px4_msgs::msg::VehicleRatesSetpoint::ConstSharedPtr msg) {
    omega_sp_ << msg->roll, msg->pitch, msg->yaw;
    have_rate_setpoint_ = true;

    if (msg->reset_integral) {
      rate_controller_.reset();
    }
  }

  void ShuttleRateController::on_force_setpoint_callback(
    const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
    force_sp_ << msg->vector.x, msg->vector.y, msg->vector.z;
    have_force_setpoint_ = true;
  }

  void ShuttleRateController::update_rate_controller(const double dt) {
    if (!have_rate_setpoint_ || !have_angular_velocity_ || !have_force_setpoint_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "ShuttleRateController waiting for inputs: rate_setpoint=%s state_angular_velocity=%s "
        "force_setpoint=%s",
        have_rate_setpoint_ ? "received" : "missing",
        have_angular_velocity_ ? "received" : "missing",
        have_force_setpoint_ ? "received" : "missing");
      return;
    }

    const Eigen::Vector3d torque_sp_Nm = rate_controller_.update(dt, omega_sp_, omega_measured_);
    publish_wrench(torque_sp_Nm, force_sp_);

    const RateController::Debug& debug = rate_controller_.debug();
    std::ostringstream log;
    log << "ShuttleRateController rate PID\n"
        << "  axes              roll    pitch      yaw\n"
        << "  omega_sp_rad_s    " << vector_to_string(omega_sp_) << "\n"
        << "  omega_meas_rad_s  " << vector_to_string(omega_measured_) << "\n"
        << "  error_rad_s       " << vector_to_string(debug.error) << "\n"
        << "  integral          " << vector_to_string(debug.integral) << "\n"
        << "  derivative        " << vector_to_string(debug.derivative) << "\n"
        << "  torque_raw_Nm     " << vector_to_string(debug.torque_unsaturated_Nm) << "\n"
        << "  torque_cmd_Nm     " << vector_to_string(debug.torque_saturated_Nm) << "\n"
        << "  force_sp_N        " << vector_to_string(force_sp_) << "\n"
        << "  wrench_cmd        " << vector_to_string(wrench_);
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, log.str());
  }

  void ShuttleRateController::publish_wrench(
    const Eigen::Vector3d& torque_sp_Nm,
    const Eigen::Vector3d& force_sp_N) {
    wrench_ << torque_sp_Nm.x(), torque_sp_Nm.y(), torque_sp_Nm.z(), force_sp_N.z();
    have_wrench_ = true;

    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = "shuttle_body";
    msg.wrench.torque.x = torque_sp_Nm.x();
    msg.wrench.torque.y = torque_sp_Nm.y();
    msg.wrench.torque.z = torque_sp_Nm.z();
    msg.wrench.force.x = force_sp_N.x();
    msg.wrench.force.y = force_sp_N.y();
    msg.wrench.force.z = force_sp_N.z();
    wrench_pub_->publish(msg);
  }

  Eigen::Matrix<double, 4, 4> ShuttleRateController::build_physical_effectiveness_matrix() const {
    Eigen::Matrix<double, 4, 4> effectiveness = Eigen::Matrix<double, 4, 4>::Zero();

    for (int i = 0; i < 4; ++i) {
      effectiveness(0, i) = quad_positions_(1, i);
      effectiveness(1, i) = quad_positions_(0, i);
      effectiveness(2, i) = quad_sigmas_(i) * quad_moment_constant_;
      effectiveness(3, i) = -1.0;
    }

    return effectiveness;
  }

  Eigen::Matrix<double, 4, 1> ShuttleRateController::allocate_thrust(
    const Eigen::Matrix<double, 4, 4>& effectiveness,
    const Eigen::Matrix<double, 4, 1>& u_phys) const {
    return effectiveness.inverse() * u_phys;
  }

  Eigen::Matrix<double, 4, 1> ShuttleRateController::thrust_to_motor_speed(
    const Eigen::Matrix<double, 4, 1>& thrust) const {
    Eigen::Matrix<double, 4, 1> omega;
    for (int i = 0; i < 4; ++i) {
      // Use absolute value of thrust for calculating speed, as thrust can be negative.
      omega(i) = std::sqrt(std::abs(thrust(i)) / motor_constants_(i));
    }
    return omega;
  }

  Eigen::Matrix<double, 4, 1> ShuttleRateController::thrust_to_normalized_motor_command(
    const Eigen::Matrix<double, 4, 1>& thrust) const {
    Eigen::Matrix<double, 4, 1> motor_command;
    // This logic assumes symmetric positive and negative thrust limits,
    // which is a reasonable simplification for this type of controller.
    for (int i = 0; i < 4; ++i) {
      const double thrust_max =
        motor_constants_(i) * max_rot_velocities_(i) * max_rot_velocities_(i);
      const double thrust_clamped = std::clamp(thrust(i), -thrust_max, thrust_max);
      const double thrust_normalized = thrust_clamped / thrust_max;
      motor_command(i) =
        std::copysign(std::sqrt(std::abs(thrust_normalized)), thrust_normalized);
    }
    return motor_command;
  }

  void ShuttleRateController::publish_actuators() {
    if (!have_wrench_) {
      return;
    }

    const uint64_t now_us = node_->get_clock()->now().nanoseconds() / 1000;

    const Eigen::Matrix<double, 4, 4> effectiveness = build_physical_effectiveness_matrix();
    const Eigen::Matrix<double, 4, 1> delta_thrust = allocate_thrust(effectiveness, wrench_);
    const Eigen::Matrix<double, 4, 1> thrust_before_saturation = thrust_trim_ + delta_thrust;
    Eigen::Matrix<double, 4, 1> thrust_after_saturation;
    bool clipped = false;
    bool negative_thrust_requested = contains_negative_thrust(thrust_before_saturation);
    for (int i = 0; i < 4; ++i) {
      const double thrust_max =
        motor_constants_(i) * max_rot_velocities_(i) * max_rot_velocities_(i);
      // Allow negative thrust for reversible motors.
      // The range is symmetric around zero.
      thrust_after_saturation(i) = std::clamp(thrust_before_saturation(i), -thrust_max, thrust_max);
      clipped = clipped || thrust_after_saturation(i) != thrust_before_saturation(i);
    }
    const Eigen::Matrix<double, 4, 1> omega_cmd = thrust_to_motor_speed(thrust_after_saturation);
    const Eigen::Matrix<double, 4, 1> actuator_commands =
      thrust_to_normalized_motor_command(thrust_after_saturation);
    const Eigen::Matrix<double, 4, 1> achieved_wrench =
      effectiveness * (thrust_after_saturation - thrust_trim_); // This is delta thrust achieved
    const Eigen::Matrix<double, 4, 1> wrench_error = wrench_ - achieved_wrench;

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

    std::ostringstream allocation_log;
    allocation_log << "ShuttleRateController physical allocation" << (clipped ? " clipped" : "")
                   << "\n";
    if (negative_thrust_requested) {
      allocation_log
        << "  note: allocation requested negative thrust; non-reversible motors clamp it to 0 N\n";
    }
    allocation_log << "  wrench axes       [Mx My Mz Fz]\n"
                   << "  requested_Nm_N    " << vector_to_string(wrench_) << "\n"
                   << "  allocated_Nm_N    " << vector_to_string(achieved_wrench) << "\n"
                   << "  error_Nm_N        " << vector_to_string(wrench_error) << "\n"
                   << actuator_allocation_table(
                        thrust_before_saturation,
                        thrust_after_saturation,
                        omega_cmd,
                        actuator_commands);
    RCLCPP_INFO_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000, allocation_log.str());

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "ShuttleRateController actuator commands: q=[%.3f %.3f %.3f %.3f]",
      shuttle_motors_msg_.control[0],
      shuttle_motors_msg_.control[1],
      shuttle_motors_msg_.control[2],
      shuttle_motors_msg_.control[3]);
  }

  void ShuttleRateController::publish_offboard_heartbeat() {
    publish_offboard_mode();
    publish_offboard_request(mode_pub_, vehicle_id_);
  }

  void ShuttleRateController::publish_offboard_mode() {
    offboard_msg_.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    offboard_msg_.position = false;
    offboard_msg_.velocity = false;
    offboard_msg_.acceleration = false;
    offboard_msg_.attitude = false;
    offboard_msg_.body_rate = false;
    offboard_msg_.thrust_and_torque = false;
    offboard_msg_.direct_actuator = true;

    offboard_pub_->publish(offboard_msg_);
  }

  void ShuttleRateController::publish_offboard_request(
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
