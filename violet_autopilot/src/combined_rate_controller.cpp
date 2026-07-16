#include "combined_rate_controller.hpp"

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

    template<typename Derived>
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
      const Eigen::Matrix<double, 5, 1>& thrust_before_saturation,
      const Eigen::Matrix<double, 5, 1>& thrust_after_saturation,
      const Eigen::Matrix<double, 5, 1>& omega_cmd,
      const Eigen::Matrix<double, 5, 1>& actuator_commands) {
      static constexpr const char* labels[5] = {"q1", "q2", "q3", "q4", "prop"};

      std::ostringstream stream;
      stream << std::fixed << std::setprecision(3);
      stream << "  actuator   T_req_N   T_cmd_N   clipped_N   omega_rad_s   cmd\n";
      for (int i = 0; i < 5; ++i) {
        stream
          << "  " << std::setw(8) << labels[i]
          << std::setw(10) << thrust_before_saturation(i)
          << std::setw(10) << thrust_after_saturation(i)
          << std::setw(12) << thrust_after_saturation(i) - thrust_before_saturation(i)
          << std::setw(14) << omega_cmd(i)
          << std::setw(8) << actuator_commands(i)
          << "\n";
      }
      return stream.str();
    }

    bool contains_negative_thrust(const Eigen::Matrix<double, 5, 1>& thrust) {
      for (Eigen::Index i = 0; i < thrust.size(); ++i) {
        if (thrust(i) < 0.0) {
          return true;
        }
      }
      return false;
    }
  }

  RateController::GainForm RateController::gain_form_from_string(
    const std::string& value,
    bool* valid) {
    if (value == "standard") {
      if (valid != nullptr) {
        *valid = true;
      }
      return GainForm::Standard;
    }
    if (valid != nullptr) {
      *valid = value == "parallel";
    }
    return GainForm::Parallel;
  }

  void RateController::configure(const Config& config) {
    config_ = config;
    reset();
  }

  void RateController::reset() {
    integral_.setZero();
    previous_omega_.setZero();
    debug_ = Debug{};
    have_previous_omega_ = false;
  }

  Eigen::Vector3d RateController::update(
    double dt,
    const Eigen::Vector3d& omega_sp,
    const Eigen::Vector3d& omega_measured) {
    if (dt <= 0.0 || !std::isfinite(dt)) {
      dt = 0.01;
    }

    const Eigen::Vector3d error = omega_sp - omega_measured;
    Eigen::Vector3d derivative = Eigen::Vector3d::Zero();
    if (have_previous_omega_) {
      derivative = -(omega_measured - previous_omega_) / dt;
    }
    previous_omega_ = omega_measured;
    have_previous_omega_ = true;

    const Eigen::Vector3d candidate_integral = integral_ + error * dt;
    Eigen::Vector3d clamped_candidate_integral = candidate_integral;
    for (int axis = 0; axis < 3; ++axis) {
      const auto& limit = config_.integral_limits[axis];
      clamped_candidate_integral(axis) =
        std::clamp(clamped_candidate_integral(axis), limit.min, limit.max);
    }

    const Eigen::Vector3d unsaturated_with_candidate =
      output_from_integral(error, clamped_candidate_integral, derivative);
    Eigen::Vector3d saturated = unsaturated_with_candidate;
    Eigen::Vector3d accepted_integral = integral_;

    for (int axis = 0; axis < 3; ++axis) {
      const auto& limit = config_.output_limits[axis];
      saturated(axis) = std::clamp(saturated(axis), limit.min, limit.max);

      const bool inside =
        unsaturated_with_candidate(axis) >= limit.min &&
        unsaturated_with_candidate(axis) <= limit.max;
      const bool high_and_reducing =
        unsaturated_with_candidate(axis) > limit.max && error(axis) < 0.0;
      const bool low_and_reducing =
        unsaturated_with_candidate(axis) < limit.min && error(axis) > 0.0;

      if (inside || high_and_reducing || low_and_reducing) {
        accepted_integral(axis) = clamped_candidate_integral(axis);
      }
    }
    integral_ = accepted_integral;

    const Eigen::Vector3d unsaturated = output_from_integral(error, integral_, derivative);
    for (int axis = 0; axis < 3; ++axis) {
      const auto& limit = config_.output_limits[axis];
      saturated(axis) = std::clamp(unsaturated(axis), limit.min, limit.max);
    }

    debug_.error = error;
    debug_.integral = integral_;
    debug_.derivative = derivative;
    debug_.torque_unsaturated_Nm = unsaturated;
    debug_.torque_saturated_Nm = saturated;
    debug_.omega_measured_filtered = omega_measured;

    return saturated;
  }

  const RateController::Debug& RateController::debug() const {
    return debug_;
  }

  const RateController::Config& RateController::config() const {
    return config_;
  }

  double RateController::axis_output(
    const int axis,
    const double error,
    const double integral,
    const double derivative) const {
    const AxisGains gains[3] = {config_.roll, config_.pitch, config_.yaw};
    const AxisGains& gain = gains[axis];

    if (config_.gain_form == GainForm::Standard) {
      return gain.kp * (error + gain.ki * integral + gain.kd * derivative);
    }
    return gain.kp * error + gain.ki * integral + gain.kd * derivative;
  }

  Eigen::Vector3d RateController::output_from_integral(
    const Eigen::Vector3d& error,
    const Eigen::Vector3d& integral,
    const Eigen::Vector3d& derivative) const {
    Eigen::Vector3d output;
    for (int axis = 0; axis < 3; ++axis) {
      output(axis) = axis_output(axis, error(axis), integral(axis), derivative(axis));
    }
    return output;
  }

  CombinedRateController::~CombinedRateController() {}

  void CombinedRateController::initialize() {
    node_->declare_parameter<std::string>(
      "controllers.combinedratecontroller.subscribers.wrench",
      "fmu/in/combined_wrench");
    node_->declare_parameter<std::string>(
      "controllers.combinedratecontroller.publishers.shuttle_motors",
      "fmu/in/actuator_motors");
    node_->declare_parameter<std::string>(
      "controllers.combinedratecontroller.fixed_wing_namespace",
      "drone2");
    node_->declare_parameter<int>(
      "controllers.combinedratecontroller.fixed_wing_vehicle_id",
      2);
    node_->declare_parameter<double>(
      "controllers.combinedratecontroller.limits.motor_min",
      -1.0);
    node_->declare_parameter<double>(
      "controllers.combinedratecontroller.limits.motor_max",
      1.0);
    node_->declare_parameter<double>(
      "controllers.combinedratecontroller.limits.servo_min",
      -1.0);
    node_->declare_parameter<double>(
      "controllers.combinedratecontroller.limits.servo_max",
      1.0);
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.wrench_scales",
      {1.0, 1.0, 1.0, 1.0, 1.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.quad_positions",
      {
        0.248, -0.248, -0.51,
       -0.248,  0.248, -0.51,
        0.248,  0.248, -0.51,
       -0.248, -0.248, -0.51
      });
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.quad_sigmas",
      {1.0, 1.0, -1.0, -1.0});
    node_->declare_parameter<double>(
      "controllers.combinedratecontroller.quad_moment_constant",
      0.016);
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.fixed_wing_prop_position",
      {0.345, 0.0, 0.306});
    node_->declare_parameter<double>(
      "controllers.combinedratecontroller.fixed_wing_sigma",
      -1.0);
    node_->declare_parameter<double>(
      "controllers.combinedratecontroller.fixed_wing_moment_constant",
      0.01);
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.motor_constants",
      {1.709716e-05, 1.709716e-05, 1.709716e-05, 1.709716e-05, 8.54858e-06});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.max_rot_velocities",
      {1400.0, 1400.0, 1400.0, 1400.0, 3500.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.thrust_trim",
      {0.0, 0.0, 0.0, 0.0, 0.0});
    node_->declare_parameter<double>("controllers.combinedratecontroller.gains.roll.kp", 0.25);
    node_->declare_parameter<double>("controllers.combinedratecontroller.gains.roll.ki", 0.0);
    node_->declare_parameter<double>("controllers.combinedratecontroller.gains.roll.kd", 0.0);
    node_->declare_parameter<double>("controllers.combinedratecontroller.gains.pitch.kp", 0.25);
    node_->declare_parameter<double>("controllers.combinedratecontroller.gains.pitch.ki", 0.0);
    node_->declare_parameter<double>("controllers.combinedratecontroller.gains.pitch.kd", 0.0);
    node_->declare_parameter<double>("controllers.combinedratecontroller.gains.yaw.kp", 0.10);
    node_->declare_parameter<double>("controllers.combinedratecontroller.gains.yaw.ki", 0.0);
    node_->declare_parameter<double>("controllers.combinedratecontroller.gains.yaw.kd", 0.0);
    node_->declare_parameter<std::string>("controllers.combinedratecontroller.gain_form", "parallel");
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.output_limits.roll",
      {-2.0, 2.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.output_limits.pitch",
      {-2.0, 2.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.output_limits.yaw",
      {-1.0, 1.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.integral_limits.roll",
      {-5.0, 5.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.integral_limits.pitch",
      {-5.0, 5.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedratecontroller.integral_limits.yaw",
      {-5.0, 5.0});
    node_->declare_parameter<std::string>(
      "controllers.combinedratecontroller.subscribers.rate_setpoint",
      "controller/in/rate_setpoint");
    node_->declare_parameter<std::string>(
      "controllers.combinedratecontroller.subscribers.force_setpoint",
      "controller/in/force_setpoint");
    node_->declare_parameter<std::string>(
      "controllers.combinedratecontroller.publishers.wrench",
      "fmu/in/combined_wrench");

    const std::string wrench_topic =
      node_->get_parameter("controllers.combinedratecontroller.subscribers.wrench").as_string();
    const std::string shuttle_motors_topic =
      node_->get_parameter("controllers.combinedratecontroller.publishers.shuttle_motors").as_string();
    const std::string rate_setpoint_topic =
      node_->get_parameter("controllers.combinedratecontroller.subscribers.rate_setpoint").as_string();
    const std::string force_setpoint_topic =
      node_->get_parameter("controllers.combinedratecontroller.subscribers.force_setpoint").as_string();
    const std::string rate_wrench_topic =
      node_->get_parameter("controllers.combinedratecontroller.publishers.wrench").as_string();

    fixed_wing_namespace_ =
      node_->get_parameter("controllers.combinedratecontroller.fixed_wing_namespace").as_string();
    fixed_wing_vehicle_id_ =
      node_->get_parameter("controllers.combinedratecontroller.fixed_wing_vehicle_id").as_int();
    motor_min_ =
      node_->get_parameter("controllers.combinedratecontroller.limits.motor_min").as_double();
    motor_max_ =
      node_->get_parameter("controllers.combinedratecontroller.limits.motor_max").as_double();
    servo_min_ =
      node_->get_parameter("controllers.combinedratecontroller.limits.servo_min").as_double();
    servo_max_ =
      node_->get_parameter("controllers.combinedratecontroller.limits.servo_max").as_double();
    quad_moment_constant_ =
      node_->get_parameter("controllers.combinedratecontroller.quad_moment_constant").as_double();
    fixed_wing_sigma_ =
      node_->get_parameter("controllers.combinedratecontroller.fixed_wing_sigma").as_double();
    fixed_wing_moment_constant_ =
      node_->get_parameter("controllers.combinedratecontroller.fixed_wing_moment_constant").as_double();

    RateController::Config rate_config;
    rate_config.roll.kp = node_->get_parameter("controllers.combinedratecontroller.gains.roll.kp").as_double();
    rate_config.roll.ki = node_->get_parameter("controllers.combinedratecontroller.gains.roll.ki").as_double();
    rate_config.roll.kd = node_->get_parameter("controllers.combinedratecontroller.gains.roll.kd").as_double();
    rate_config.pitch.kp = node_->get_parameter("controllers.combinedratecontroller.gains.pitch.kp").as_double();
    rate_config.pitch.ki = node_->get_parameter("controllers.combinedratecontroller.gains.pitch.ki").as_double();
    rate_config.pitch.kd = node_->get_parameter("controllers.combinedratecontroller.gains.pitch.kd").as_double();
    rate_config.yaw.kp = node_->get_parameter("controllers.combinedratecontroller.gains.yaw.kp").as_double();
    rate_config.yaw.ki = node_->get_parameter("controllers.combinedratecontroller.gains.yaw.ki").as_double();
    rate_config.yaw.kd = node_->get_parameter("controllers.combinedratecontroller.gains.yaw.kd").as_double();
    bool valid_gain_form = false;
    rate_config.gain_form = RateController::gain_form_from_string(
      node_->get_parameter("controllers.combinedratecontroller.gain_form").as_string(),
      &valid_gain_form);
    if (!valid_gain_form) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Invalid controllers.combinedratecontroller.gain_form; using parallel");
    }
    rate_config.output_limits[0] = limit_parameter_or_default(
      node_->get_parameter("controllers.combinedratecontroller.output_limits.roll").as_double_array(),
      {-2.0, 2.0},
      node_->get_logger(),
      "controllers.combinedratecontroller.output_limits.roll");
    rate_config.output_limits[1] = limit_parameter_or_default(
      node_->get_parameter("controllers.combinedratecontroller.output_limits.pitch").as_double_array(),
      {-2.0, 2.0},
      node_->get_logger(),
      "controllers.combinedratecontroller.output_limits.pitch");
    rate_config.output_limits[2] = limit_parameter_or_default(
      node_->get_parameter("controllers.combinedratecontroller.output_limits.yaw").as_double_array(),
      {-1.0, 1.0},
      node_->get_logger(),
      "controllers.combinedratecontroller.output_limits.yaw");
    rate_config.integral_limits[0] = limit_parameter_or_default(
      node_->get_parameter("controllers.combinedratecontroller.integral_limits.roll").as_double_array(),
      {-5.0, 5.0},
      node_->get_logger(),
      "controllers.combinedratecontroller.integral_limits.roll");
    rate_config.integral_limits[1] = limit_parameter_or_default(
      node_->get_parameter("controllers.combinedratecontroller.integral_limits.pitch").as_double_array(),
      {-5.0, 5.0},
      node_->get_logger(),
      "controllers.combinedratecontroller.integral_limits.pitch");
    rate_config.integral_limits[2] = limit_parameter_or_default(
      node_->get_parameter("controllers.combinedratecontroller.integral_limits.yaw").as_double_array(),
      {-5.0, 5.0},
      node_->get_logger(),
      "controllers.combinedratecontroller.integral_limits.yaw");
    rate_controller_.configure(rate_config);

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
      node_->get_parameter("controllers.combinedratecontroller.wrench_scales").as_double_array(),
      default_wrench_scale,
      node_->get_logger(),
      "controllers.combinedratecontroller.wrench_scales");
    const Eigen::Matrix<double, Eigen::Dynamic, 1> quad_position_values =
      vector_parameter_or_default(
        node_->get_parameter("controllers.combinedratecontroller.quad_positions").as_double_array(),
        default_quad_positions,
        node_->get_logger(),
        "controllers.combinedratecontroller.quad_positions");
    for (int i = 0; i < 4; ++i) {
      quad_positions_.col(i) = quad_position_values.segment<3>(3 * i);
    }
    quad_sigmas_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combinedratecontroller.quad_sigmas").as_double_array(),
      default_quad_sigmas,
      node_->get_logger(),
      "controllers.combinedratecontroller.quad_sigmas");
    fixed_wing_prop_position_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combinedratecontroller.fixed_wing_prop_position").as_double_array(),
      default_fixed_wing_position,
      node_->get_logger(),
      "controllers.combinedratecontroller.fixed_wing_prop_position");
    motor_constants_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combinedratecontroller.motor_constants").as_double_array(),
      default_motor_constants,
      node_->get_logger(),
      "controllers.combinedratecontroller.motor_constants");
    max_rot_velocities_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combinedratecontroller.max_rot_velocities").as_double_array(),
      default_max_rot_velocities,
      node_->get_logger(),
      "controllers.combinedratecontroller.max_rot_velocities");
    thrust_trim_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combinedratecontroller.thrust_trim").as_double_array(),
      default_thrust_trim,
      node_->get_logger(),
      "controllers.combinedratecontroller.thrust_trim");

    fixed_wing_namespace_.erase(0, fixed_wing_namespace_.find_first_not_of('/'));
    fixed_wing_namespace_.erase(fixed_wing_namespace_.find_last_not_of('/') + 1);
    const std::string fixed_wing_prefix = "/" + fixed_wing_namespace_;

    wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      wrench_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&CombinedRateController::on_wrench_callback, this, std::placeholders::_1));
    rate_setpoint_sub_ = node_->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
      rate_setpoint_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&CombinedRateController::on_rate_setpoint_callback, this, std::placeholders::_1));
    force_setpoint_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      force_setpoint_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&CombinedRateController::on_force_setpoint_callback, this, std::placeholders::_1));
    wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      rate_wrench_topic,
      rclcpp::SensorDataQoS());

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
      std::chrono::milliseconds(20),
      std::bind(&CombinedRateController::publish_offboard_heartbeat, this));

    RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "CombinedRateController initialized. Wrench topic: "
        << wrench_topic << ", rate setpoint topic: "
        << rate_setpoint_topic << ", force setpoint topic: "
        << force_setpoint_topic << ", fixed-wing namespace: /"
        << fixed_wing_namespace_);
  }

  void CombinedRateController::set_position(const double dt, const Eigen::Vector3d& p) {
    (void)dt;
    (void)p;
    publish_actuators();
  }

  void CombinedRateController::set_attitude(
    const double dt,
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& v) {
    (void)dt;
    (void)p;
    (void)v;
    publish_actuators();
  }

  void CombinedRateController::set_attitude_rate(
    const double dt,
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& v,
    const Eigen::Vector3d& eta) {
    (void)p;
    (void)v;
    omega_measured_ = eta;
    have_angular_velocity_ = true;
    update_rate_controller(dt);
    publish_actuators();
  }

  void CombinedRateController::set_path(const int type, const double* path) {
    (void)type;
    (void)path;
  }

  void CombinedRateController::reset() {
    rate_controller_.reset();
    omega_sp_.setZero();
    omega_measured_.setZero();
    force_sp_.setZero();
    have_rate_setpoint_ = false;
    have_angular_velocity_ = false;
    have_force_setpoint_ = false;
    have_wrench_ = false;
  }

  void CombinedRateController::on_wrench_callback(
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
      "CombinedRateController received wrench: M=[%.3f %.3f %.3f], Fx=%.3f, Fz=%.3f, ignored Fy=%.3f",
      wrench_(0),
      wrench_(1),
      wrench_(2),
      wrench_(3),
      wrench_(4),
      msg->wrench.force.y);
  }

  void CombinedRateController::on_rate_setpoint_callback(
    const px4_msgs::msg::VehicleRatesSetpoint::ConstSharedPtr msg) {
    omega_sp_ << msg->roll, msg->pitch, msg->yaw;
    have_rate_setpoint_ = true;

    if (msg->reset_integral) {
      rate_controller_.reset();
    }
  }

  void CombinedRateController::on_force_setpoint_callback(
    const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
    force_sp_ << msg->vector.x, msg->vector.y, msg->vector.z;
    have_force_setpoint_ = true;
  }

  void CombinedRateController::update_rate_controller(const double dt) {
    if (!have_rate_setpoint_ || !have_angular_velocity_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "CombinedRateController waiting for inputs: rate_setpoint=%s state_angular_velocity=%s",
        have_rate_setpoint_ ? "received" : "missing",
        have_angular_velocity_ ? "received" : "missing");
      return;
    }

    const Eigen::Vector3d torque_sp_Nm =
      rate_controller_.update(dt, omega_sp_, omega_measured_);
    publish_wrench(torque_sp_Nm, force_sp_);

    const RateController::Debug& debug = rate_controller_.debug();
    std::ostringstream log;
    log << "CombinedRateController rate PID\n"
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

  void CombinedRateController::publish_wrench(
    const Eigen::Vector3d& torque_sp_Nm,
    const Eigen::Vector3d& force_sp_N) {
    wrench_ <<
      torque_sp_Nm.x(),
      torque_sp_Nm.y(),
      torque_sp_Nm.z(),
      force_sp_N.x(),
      force_sp_N.z();
    have_wrench_ = true;

    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = "combined_body";
    msg.wrench.torque.x = torque_sp_Nm.x();
    msg.wrench.torque.y = torque_sp_Nm.y();
    msg.wrench.torque.z = torque_sp_Nm.z();
    msg.wrench.force.x = force_sp_N.x();
    msg.wrench.force.y = force_sp_N.y();
    msg.wrench.force.z = force_sp_N.z();
    wrench_pub_->publish(msg);
  }

  Eigen::Matrix<double, 5, 5> CombinedRateController::build_physical_effectiveness_matrix() const {
    Eigen::Matrix<double, 5, 5> effectiveness = Eigen::Matrix<double, 5, 5>::Zero();

    for (int i = 0; i < 4; ++i) {
      // Motor positions come from the Gazebo model frame used by the shuttle SDF.
      // The simulated body-rate signs match these moment arms for roll/pitch.
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

  Eigen::Matrix<double, 5, 1> CombinedRateController::normalized_wrench_to_physical(
    const Eigen::Matrix<double, 5, 1>& u_command) const {
    (void)wrench_scale_;
    return u_command;
  }

  Eigen::Matrix<double, 5, 1> CombinedRateController::allocate_thrust(
    const Eigen::Matrix<double, 5, 5>& effectiveness,
    const Eigen::Matrix<double, 5, 1>& u_phys) const {
    return effectiveness.inverse() * u_phys;
  }

  Eigen::Matrix<double, 5, 1> CombinedRateController::thrust_to_motor_speed(
    const Eigen::Matrix<double, 5, 1>& thrust) const {
    Eigen::Matrix<double, 5, 1> omega;
    for (int i = 0; i < 5; ++i) {
      omega(i) = std::sqrt(std::max(0.0, thrust(i)) / motor_constants_(i));
    }
    return omega;
  }

  Eigen::Matrix<double, 5, 1> CombinedRateController::thrust_to_normalized_motor_command(
    const Eigen::Matrix<double, 5, 1>& thrust) const {
    Eigen::Matrix<double, 5, 1> motor_command;
    for (int i = 0; i < 5; ++i) {
      const double thrust_max =
        motor_constants_(i) * max_rot_velocities_(i) * max_rot_velocities_(i);
      motor_command(i) = std::sqrt(std::clamp(thrust(i), 0.0, thrust_max) / thrust_max);
    }
    return motor_command;
  }

  void CombinedRateController::publish_actuators() {
    if (!have_wrench_) {
      return;
    }

    const uint64_t now_us = node_->get_clock()->now().nanoseconds() / 1000;

    const Eigen::Matrix<double, 5, 5> effectiveness = build_physical_effectiveness_matrix();
    const Eigen::Matrix<double, 5, 1> u_command = wrench_;
    const Eigen::Matrix<double, 5, 1> u_phys = normalized_wrench_to_physical(u_command);
    const Eigen::Matrix<double, 5, 1> delta_thrust = allocate_thrust(effectiveness, u_phys);
    const Eigen::Matrix<double, 5, 1> thrust_before_saturation = thrust_trim_ + delta_thrust;
    Eigen::Matrix<double, 5, 1> thrust_after_saturation;
    bool clipped = false;
    bool negative_thrust_requested = contains_negative_thrust(thrust_before_saturation);
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

    std::ostringstream allocation_log;
    allocation_log << "CombinedRateController physical allocation" << (clipped ? " clipped" : "") << "\n";
    if (negative_thrust_requested) {
      allocation_log
        << "  note: allocation requested negative thrust; non-reversible motors clamp it to 0 N\n";
    }
    allocation_log
      << "  wrench axes       [Mx My Mz Fx Fz]\n"
      << "  requested_Nm_N    " << vector_to_string(u_command) << "\n"
      << "  allocated_Nm_N    " << vector_to_string(achieved_wrench) << "\n"
      << "  error_Nm_N        " << vector_to_string(wrench_error) << "\n"
      << actuator_allocation_table(
        thrust_before_saturation,
        thrust_after_saturation,
        omega_cmd,
        actuator_commands);
    RCLCPP_INFO_STREAM_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      allocation_log.str());

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "CombinedRateController actuator commands: q=[%.3f %.3f %.3f %.3f], prop=%.3f, surf=[%.3f %.3f %.3f %.3f]",
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

  void CombinedRateController::publish_offboard_heartbeat() {
    publish_offboard_mode();
    publish_offboard_request(shuttle_mode_pub_, vehicle_id_);
    publish_offboard_request(fixed_wing_mode_pub_, fixed_wing_vehicle_id_);
  }

  void CombinedRateController::publish_offboard_mode() {
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

  void CombinedRateController::publish_offboard_request(
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
