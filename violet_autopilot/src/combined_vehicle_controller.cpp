#include "combined_vehicle_controller.hpp"

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

    Eigen::Vector3d lemniscate_derivative(const double a, const double gamma) {
      const double sine = std::sin(gamma);
      const double denominator = 1.0 + sine * sine;
      Eigen::Vector3d derivative;
      derivative <<
        a * sine * (sine * sine - 3.0) / (denominator * denominator),
        a * (1.0 - 3.0 * sine * sine) / (denominator * denominator),
        0.0;
      return derivative;
    }

    double signed_horizontal_curvature(
      const Eigen::Vector3d& dpd_dgamma,
      const Eigen::Vector3d& d2pd_dgamma2) {
      const double horizontal_derivative_squared =
        dpd_dgamma.x() * dpd_dgamma.x() +
        dpd_dgamma.y() * dpd_dgamma.y();
      if (horizontal_derivative_squared <= 1e-12 ||
          !std::isfinite(horizontal_derivative_squared)) {
        return 0.0;
      }

      const double denominator =
        horizontal_derivative_squared * std::sqrt(horizontal_derivative_squared);
      const double curvature =
        (dpd_dgamma.x() * d2pd_dgamma2.y() -
         dpd_dgamma.y() * d2pd_dgamma2.x()) / denominator;
      return std::isfinite(curvature) ? curvature : 0.0;
    }

  }


  CombinedVehicleController::~CombinedVehicleController() {}

  void CombinedVehicleController::initialize() {
    node_->declare_parameter<std::string>(
      "controllers.combinedvehiclecontroller.subscribers.wrench",
      "fmu/in/combined_wrench");
    node_->declare_parameter<std::string>(
      "controllers.combinedvehiclecontroller.publishers.shuttle_motors",
      "fmu/in/actuator_motors");
    node_->declare_parameter<std::string>(
      "controllers.combinedvehiclecontroller.fixed_wing_namespace",
      "drone2");
    node_->declare_parameter<int>(
      "controllers.combinedvehiclecontroller.fixed_wing_vehicle_id",
      2);
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.limits.motor_min",
      -1.0);
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.limits.motor_max",
      1.0);
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.limits.servo_min",
      -1.0);
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.limits.servo_max",
      1.0);
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.wrench_scales",
      {1.0, 1.0, 1.0, 1.0, 1.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.quad_positions",
      {
        0.248, -0.248, -0.51,
       -0.248,  0.248, -0.51,
        0.248,  0.248, -0.51,
       -0.248, -0.248, -0.51
      });
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.quad_sigmas",
      {1.0, 1.0, -1.0, -1.0});
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.quad_moment_constant",
      0.016);
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.fixed_wing_prop_position",
      {0.345, 0.0, 0.306});
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.fixed_wing_sigma",
      -1.0);
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.fixed_wing_moment_constant",
      0.01);
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.motor_constants",
      {1.709716e-05, 1.709716e-05, 1.709716e-05, 1.709716e-05, 8.54858e-06});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.max_rot_velocities",
      {1400.0, 1400.0, 1400.0, 1400.0, 3500.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.thrust_trim",
      {0.0, 0.0, 0.0, 0.0, 0.0});
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.roll.kp", 0.25);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.roll.ki", 0.0);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.roll.kd", 0.0);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.pitch.kp", 0.25);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.pitch.ki", 0.0);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.pitch.kd", 0.0);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.yaw.kp", 0.10);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.yaw.ki", 0.0);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.yaw.kd", 0.0);
    node_->declare_parameter<std::string>("controllers.combinedvehiclecontroller.gain_form", "parallel");
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.output_limits.roll",
      {-2.0, 2.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.output_limits.pitch",
      {-2.0, 2.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.output_limits.yaw",
      {-1.0, 1.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.integral_limits.roll",
      {-5.0, 5.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.integral_limits.pitch",
      {-5.0, 5.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.integral_limits.yaw",
      {-5.0, 5.0});
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.attitude_gains.roll", 1.0);
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.attitude_gains.pitch", 1.0);
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.attitude_gains.yaw", 1.0);
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.attitude_rate_limits",
      {1.0, 1.0, 1.0});
    node_->declare_parameter<std::string>(
      "controllers.combinedvehiclecontroller.state_topic",
      "fmu/telemetry/state");
    node_->declare_parameter<std::string>(
      "controllers.combinedvehiclecontroller.publishers.wrench",
      "fmu/in/combined_wrench");
    node_->declare_parameter<std::string>(
      "controllers.combinedvehiclecontroller.publishers.achieved_wrench",
      "controller/out/achieved_wrench");
    node_->declare_parameter<std::string>(
      "controllers.combinedvehiclecontroller.publishers.attitude_reference",
      "controller/out/attitude_reference");
    node_->declare_parameter<std::string>(
      "controllers.combinedvehiclecontroller.publishers.velocity_tracking",
      "controller/out/velocity_tracking");
    node_->declare_parameter<std::string>(
      "controllers.combinedvehiclecontroller.publishers.altitude_tracking",
      "controller/out/altitude_tracking");
    node_->declare_parameter<std::string>(
      "controllers.combinedvehiclecontroller.subscribers.heading_setpoint",
      "controller/in/heading_setpoint");
    node_->declare_parameter<std::string>(
      "controllers.combinedvehiclecontroller.subscribers.airspeed_setpoint",
      "controller/in/airspeed_setpoint");
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.shuttle_mass", 3.5);
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.fixed_wing_mass", 1.5);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gravity", 9.80665);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.x.kp", 1.0);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.x.ki", 0.0);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.z.kp", 1.0);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.z.ki", 0.0);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.gains.z.kd", 0.0);
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.integral_limits.x", 20.0);
    node_->declare_parameter<double>(
      "controllers.combinedvehiclecontroller.integral_limits.z", 20.0);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.path_gains.k1", 1.0);
    node_->declare_parameter<double>("controllers.combinedvehiclecontroller.path_gains.k2", 1.0);
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.force_limits.tx", {0.0, 500.0});
    node_->declare_parameter<std::vector<double>>(
      "controllers.combinedvehiclecontroller.force_limits.tz", {0.0, 500.0});

    const std::string wrench_topic =
      node_->get_parameter("controllers.combinedvehiclecontroller.subscribers.wrench").as_string();
    const std::string shuttle_motors_topic =
      node_->get_parameter("controllers.combinedvehiclecontroller.publishers.shuttle_motors").as_string();
    const std::string state_topic =
      node_->get_parameter("controllers.combinedvehiclecontroller.state_topic").as_string();
    const std::string rate_wrench_topic =
      node_->get_parameter("controllers.combinedvehiclecontroller.publishers.wrench").as_string();
    const std::string achieved_wrench_topic = node_->get_parameter(
      "controllers.combinedvehiclecontroller.publishers.achieved_wrench").as_string();
    const std::string attitude_reference_topic = node_->get_parameter(
      "controllers.combinedvehiclecontroller.publishers.attitude_reference").as_string();
    const std::string velocity_tracking_topic = node_->get_parameter(
      "controllers.combinedvehiclecontroller.publishers.velocity_tracking").as_string();
    const std::string altitude_tracking_topic = node_->get_parameter(
      "controllers.combinedvehiclecontroller.publishers.altitude_tracking").as_string();
    const std::string heading_setpoint_topic =
      node_->get_parameter(
        "controllers.combinedvehiclecontroller.subscribers.heading_setpoint").as_string();
    const std::string airspeed_setpoint_topic =
      node_->get_parameter(
        "controllers.combinedvehiclecontroller.subscribers.airspeed_setpoint").as_string();
    shuttle_mass_ = node_->get_parameter(
      "controllers.combinedvehiclecontroller.shuttle_mass").as_double();
    fixed_wing_mass_ = node_->get_parameter(
      "controllers.combinedvehiclecontroller.fixed_wing_mass").as_double();
    gravity_ = node_->get_parameter("controllers.combinedvehiclecontroller.gravity").as_double();
    mass_ = shuttle_mass_ + fixed_wing_mass_;
    if (shuttle_mass_ <= 0.0 || fixed_wing_mass_ <= 0.0 || mass_ <= 0.0) {
      RCLCPP_WARN(
        node_->get_logger(),
        "vehicle masses must be positive; using shuttle=3.5 kg and fixed-wing=1.5 kg");
      shuttle_mass_ = 3.5;
      fixed_wing_mass_ = 1.5;
      mass_ = shuttle_mass_ + fixed_wing_mass_;
    }
    kpx_ = node_->get_parameter("controllers.combinedvehiclecontroller.gains.x.kp").as_double();
    kix_ = node_->get_parameter("controllers.combinedvehiclecontroller.gains.x.ki").as_double();
    kpz_ = node_->get_parameter("controllers.combinedvehiclecontroller.gains.z.kp").as_double();
    kiz_ = node_->get_parameter("controllers.combinedvehiclecontroller.gains.z.ki").as_double();
    kdz_ = node_->get_parameter("controllers.combinedvehiclecontroller.gains.z.kd").as_double();
    x_integral_limit_ = std::abs(node_->get_parameter(
      "controllers.combinedvehiclecontroller.integral_limits.x").as_double());
    z_integral_limit_ = std::abs(node_->get_parameter(
      "controllers.combinedvehiclecontroller.integral_limits.z").as_double());
    path_k1_ = node_->get_parameter(
      "controllers.combinedvehiclecontroller.path_gains.k1").as_double();
    path_k2_ = node_->get_parameter(
      "controllers.combinedvehiclecontroller.path_gains.k2").as_double();
    const auto tx_limits = limit_parameter_or_default(
      node_->get_parameter(
        "controllers.combinedvehiclecontroller.force_limits.tx").as_double_array(),
      {0.0, 500.0},
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.force_limits.tx");
    tx_min_ = tx_limits.min;
    tx_max_ = tx_limits.max;
    const auto tz_limits = limit_parameter_or_default(
      node_->get_parameter(
        "controllers.combinedvehiclecontroller.force_limits.tz").as_double_array(),
      {0.0, 500.0},
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.force_limits.tz");
    tz_min_ = tz_limits.min;
    tz_max_ = tz_limits.max;
    RCLCPP_INFO(
      node_->get_logger(),
      "Combined mass %.3f kg; path and velocity control use shuttle odometry directly",
      mass_);

    fixed_wing_namespace_ =
      node_->get_parameter("controllers.combinedvehiclecontroller.fixed_wing_namespace").as_string();
    fixed_wing_vehicle_id_ =
      node_->get_parameter("controllers.combinedvehiclecontroller.fixed_wing_vehicle_id").as_int();
    motor_min_ =
      node_->get_parameter("controllers.combinedvehiclecontroller.limits.motor_min").as_double();
    motor_max_ =
      node_->get_parameter("controllers.combinedvehiclecontroller.limits.motor_max").as_double();
    servo_min_ =
      node_->get_parameter("controllers.combinedvehiclecontroller.limits.servo_min").as_double();
    servo_max_ =
      node_->get_parameter("controllers.combinedvehiclecontroller.limits.servo_max").as_double();
    quad_moment_constant_ =
      node_->get_parameter("controllers.combinedvehiclecontroller.quad_moment_constant").as_double();
    fixed_wing_sigma_ =
      node_->get_parameter("controllers.combinedvehiclecontroller.fixed_wing_sigma").as_double();
    fixed_wing_moment_constant_ =
      node_->get_parameter("controllers.combinedvehiclecontroller.fixed_wing_moment_constant").as_double();

    attitude_kp_ <<
      node_->get_parameter("controllers.combinedvehiclecontroller.attitude_gains.roll").as_double(),
      node_->get_parameter("controllers.combinedvehiclecontroller.attitude_gains.pitch").as_double(),
      node_->get_parameter("controllers.combinedvehiclecontroller.attitude_gains.yaw").as_double();
    const auto attitude_rate_limits =
      node_->get_parameter(
        "controllers.combinedvehiclecontroller.attitude_rate_limits").as_double_array();
    if (attitude_rate_limits.size() == 3U) {
      attitude_rate_limits_ <<
        std::abs(attitude_rate_limits[0]),
        std::abs(attitude_rate_limits[1]),
        std::abs(attitude_rate_limits[2]);
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "attitude_rate_limits must contain roll, pitch, yaw; using [1 1 1]");
    }

    RateController::Config rate_config;
    rate_config.roll.kp = node_->get_parameter("controllers.combinedvehiclecontroller.gains.roll.kp").as_double();
    rate_config.roll.ki = node_->get_parameter("controllers.combinedvehiclecontroller.gains.roll.ki").as_double();
    rate_config.roll.kd = node_->get_parameter("controllers.combinedvehiclecontroller.gains.roll.kd").as_double();
    rate_config.pitch.kp = node_->get_parameter("controllers.combinedvehiclecontroller.gains.pitch.kp").as_double();
    rate_config.pitch.ki = node_->get_parameter("controllers.combinedvehiclecontroller.gains.pitch.ki").as_double();
    rate_config.pitch.kd = node_->get_parameter("controllers.combinedvehiclecontroller.gains.pitch.kd").as_double();
    rate_config.yaw.kp = node_->get_parameter("controllers.combinedvehiclecontroller.gains.yaw.kp").as_double();
    rate_config.yaw.ki = node_->get_parameter("controllers.combinedvehiclecontroller.gains.yaw.ki").as_double();
    rate_config.yaw.kd = node_->get_parameter("controllers.combinedvehiclecontroller.gains.yaw.kd").as_double();
    bool valid_gain_form = false;
    rate_config.gain_form = RateController::gain_form_from_string(
      node_->get_parameter("controllers.combinedvehiclecontroller.gain_form").as_string(),
      &valid_gain_form);
    if (!valid_gain_form) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Invalid controllers.combinedvehiclecontroller.gain_form; using parallel");
    }
    rate_config.output_limits[0] = limit_parameter_or_default(
      node_->get_parameter("controllers.combinedvehiclecontroller.output_limits.roll").as_double_array(),
      {-2.0, 2.0},
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.output_limits.roll");
    rate_config.output_limits[1] = limit_parameter_or_default(
      node_->get_parameter("controllers.combinedvehiclecontroller.output_limits.pitch").as_double_array(),
      {-2.0, 2.0},
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.output_limits.pitch");
    rate_config.output_limits[2] = limit_parameter_or_default(
      node_->get_parameter("controllers.combinedvehiclecontroller.output_limits.yaw").as_double_array(),
      {-1.0, 1.0},
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.output_limits.yaw");
    rate_config.integral_limits[0] = limit_parameter_or_default(
      node_->get_parameter("controllers.combinedvehiclecontroller.integral_limits.roll").as_double_array(),
      {-5.0, 5.0},
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.integral_limits.roll");
    rate_config.integral_limits[1] = limit_parameter_or_default(
      node_->get_parameter("controllers.combinedvehiclecontroller.integral_limits.pitch").as_double_array(),
      {-5.0, 5.0},
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.integral_limits.pitch");
    rate_config.integral_limits[2] = limit_parameter_or_default(
      node_->get_parameter("controllers.combinedvehiclecontroller.integral_limits.yaw").as_double_array(),
      {-5.0, 5.0},
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.integral_limits.yaw");
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
      node_->get_parameter("controllers.combinedvehiclecontroller.wrench_scales").as_double_array(),
      default_wrench_scale,
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.wrench_scales");
    const Eigen::Matrix<double, Eigen::Dynamic, 1> quad_position_values =
      vector_parameter_or_default(
        node_->get_parameter("controllers.combinedvehiclecontroller.quad_positions").as_double_array(),
        default_quad_positions,
        node_->get_logger(),
        "controllers.combinedvehiclecontroller.quad_positions");
    for (int i = 0; i < 4; ++i) {
      quad_positions_.col(i) = quad_position_values.segment<3>(3 * i);
    }
    quad_sigmas_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combinedvehiclecontroller.quad_sigmas").as_double_array(),
      default_quad_sigmas,
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.quad_sigmas");
    fixed_wing_prop_position_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combinedvehiclecontroller.fixed_wing_prop_position").as_double_array(),
      default_fixed_wing_position,
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.fixed_wing_prop_position");
    motor_constants_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combinedvehiclecontroller.motor_constants").as_double_array(),
      default_motor_constants,
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.motor_constants");
    max_rot_velocities_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combinedvehiclecontroller.max_rot_velocities").as_double_array(),
      default_max_rot_velocities,
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.max_rot_velocities");
    thrust_trim_ = vector_parameter_or_default(
      node_->get_parameter("controllers.combinedvehiclecontroller.thrust_trim").as_double_array(),
      default_thrust_trim,
      node_->get_logger(),
      "controllers.combinedvehiclecontroller.thrust_trim");

    fixed_wing_namespace_.erase(0, fixed_wing_namespace_.find_first_not_of('/'));
    fixed_wing_namespace_.erase(fixed_wing_namespace_.find_last_not_of('/') + 1);
    const std::string fixed_wing_prefix = "/" + fixed_wing_namespace_;

    wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      wrench_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&CombinedVehicleController::on_wrench_callback, this, std::placeholders::_1));
    state_sub_ = node_->create_subscription<violet_msgs::msg::State>(
      state_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&CombinedVehicleController::on_state_callback, this, std::placeholders::_1));
    heading_setpoint_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      heading_setpoint_topic,
      rclcpp::SensorDataQoS(),
      std::bind(
        &CombinedVehicleController::on_heading_setpoint_callback,
        this,
        std::placeholders::_1));
    airspeed_setpoint_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      airspeed_setpoint_topic,
      rclcpp::SensorDataQoS(),
      std::bind(
        &CombinedVehicleController::on_airspeed_setpoint_callback,
        this,
        std::placeholders::_1));
    wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      rate_wrench_topic,
      rclcpp::SensorDataQoS());
    achieved_wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      achieved_wrench_topic,
      rclcpp::SensorDataQoS());
    attitude_reference_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      attitude_reference_topic,
      rclcpp::SensorDataQoS());
    velocity_tracking_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      velocity_tracking_topic,
      rclcpp::SensorDataQoS());
    altitude_tracking_pub_ =
      node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        altitude_tracking_topic,
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
      std::bind(&CombinedVehicleController::publish_offboard_heartbeat, this));

    RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "CombinedVehicleController initialized. Wrench topic: "
        << wrench_topic << ", heading setpoint topic: "
        << heading_setpoint_topic << ", airspeed setpoint topic: "
        << airspeed_setpoint_topic << ", fixed-wing namespace: /"
        << fixed_wing_namespace_);
  }

  void CombinedVehicleController::set_position(const double dt, const Eigen::Vector3d& p) {
    (void)dt;
    (void)p;
    publish_actuators();
  }

  void CombinedVehicleController::set_attitude(
    const double dt,
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& v) {
    (void)dt;
    (void)p;
    (void)v;
    publish_actuators();
  }

  void CombinedVehicleController::set_attitude_rate(
    const double dt,
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& v,
    const Eigen::Vector3d& eta) {
    omega_measured_ = eta;
    have_angular_velocity_ = true;

    if (path_guidance_active_) {
      update_path_guidance(dt, p);
    }
    if (!have_heading_setpoint_ || !have_airspeed_setpoint_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "CombinedVehicleController waiting for guidance: heading=%s airspeed=%s",
        have_heading_setpoint_ ? "received" : "missing",
        have_airspeed_setpoint_ ? "received" : "missing");
      return;
    }

    constexpr double tx_floor = 0.5;
    const double lateral_force_feedforward =
      mass_ * airspeed_sp_ * airspeed_sp_ * local_curvature_;
    const double yaw_offset = std::asin(std::clamp(
      lateral_force_feedforward / std::max(std::abs(tx_previous_), tx_floor),
      -1.0,
      1.0));
    attitude_sp_ <<
      0.0,
      0.0,
      std::atan2(heading_sp_.y(), heading_sp_.x()) + yaw_offset;
    have_attitude_setpoint_ = true;

    geometry_msgs::msg::Vector3Stamped attitude_reference_msg;
    attitude_reference_msg.header.stamp = node_->get_clock()->now();
    attitude_reference_msg.header.frame_id = "combined_body";
    attitude_reference_msg.vector.x = attitude_sp_.x();
    attitude_reference_msg.vector.y = attitude_sp_.y();
    attitude_reference_msg.vector.z = attitude_sp_.z();
    attitude_reference_pub_->publish(attitude_reference_msg);

    // The speed PI loop uses the norm of the measured inertial velocity.
    const double yaw = attitude_measured_.z();
    const double u_measured = v.norm();
    const double lateral_velocity =
      -v.x() * std::sin(yaw) + v.y() * std::cos(yaw);
    // Forward-speed control is independent of the vertical path correction.
    const double u_cmd = airspeed_sp_;
    const double e_u = u_cmd - u_measured;
    // NED z is positive down, so e_z is positive below the desired altitude.
    const double e_z = p.z() - position_sp_.z();

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      500,
      "Heading guidance=[%.3f %.3f %.3f] Va=%.3f u_cmd=%.3f z_sp=%.3f "
      "curvature=%.4f yaw_offset=%.3f",
      heading_sp_raw_.x(), heading_sp_raw_.y(), heading_sp_raw_.z(),
      airspeed_sp_, u_cmd, position_sp_.z(), local_curvature_, yaw_offset);

    u_cmd_debug_ = u_cmd;
    u_measured_debug_ = u_measured;
    lateral_velocity_debug_ = lateral_velocity;
    inertial_velocity_debug_ = v;

    geometry_msgs::msg::Vector3Stamped velocity_tracking_msg;
    velocity_tracking_msg.header.stamp = node_->get_clock()->now();
    velocity_tracking_msg.header.frame_id = "combined_body";
    velocity_tracking_msg.vector.x = u_cmd;
    velocity_tracking_msg.vector.y = u_measured;
    velocity_tracking_msg.vector.z = lateral_velocity;
    velocity_tracking_pub_->publish(velocity_tracking_msg);

    geometry_msgs::msg::Vector3Stamped altitude_tracking_msg;
    altitude_tracking_msg.header.stamp = node_->get_clock()->now();
    altitude_tracking_msg.header.frame_id = "ned";
    altitude_tracking_msg.vector.x = position_sp_.z();
    altitude_tracking_msg.vector.y = p.z();
    altitude_tracking_msg.vector.z = e_z;
    altitude_tracking_pub_->publish(altitude_tracking_msg);

    double x_integral_candidate = x_error_integral_;
    double z_integral_candidate = z_error_integral_;
    if (dt > 0.0 && std::isfinite(dt)) {
      x_integral_candidate = std::clamp(
        x_error_integral_ + e_u * dt, -x_integral_limit_, x_integral_limit_);
      z_integral_candidate = std::clamp(
        z_error_integral_ + e_z * dt, -z_integral_limit_, z_integral_limit_);
    }

    const double tx_candidate =
      mass_ * (kpx_ * e_u + kix_ * x_integral_candidate);
    const bool tx_inside = tx_candidate >= tx_min_ && tx_candidate <= tx_max_;
    const bool tx_high_and_reducing = tx_candidate > tx_max_ && e_u < 0.0;
    const bool tx_low_and_reducing = tx_candidate < tx_min_ && e_u > 0.0;
    if (tx_inside || tx_high_and_reducing || tx_low_and_reducing) {
      x_error_integral_ = x_integral_candidate;
    }

    const double tz_candidate = mass_ *
      (gravity_ + kpz_ * e_z + kiz_ * z_integral_candidate + kdz_ * v.z());
    const bool tz_inside = tz_candidate >= tz_min_ && tz_candidate <= tz_max_;
    const bool tz_high_and_reducing = tz_candidate > tz_max_ && e_z < 0.0;
    const bool tz_low_and_reducing = tz_candidate < tz_min_ && e_z > 0.0;
    if (tz_inside || tz_high_and_reducing || tz_low_and_reducing) {
      z_error_integral_ = z_integral_candidate;
    }

    const double tx_unsaturated =
      mass_ * (kpx_ * e_u + kix_ * x_error_integral_);
    // Tz is an upward thrust magnitude. In NED, positive e_z means the
    // vehicle is below its target and positive v.z() means it is descending;
    // both conditions require more upward thrust than hover.
    const double tz_unsaturated = mass_ *
      (gravity_ + kpz_ * e_z + kiz_ * z_error_integral_ + kdz_ * v.z());
    const double tx = std::clamp(tx_unsaturated, tx_min_, tx_max_);
    const double tz = std::clamp(tz_unsaturated, tz_min_, tz_max_);
    tx_previous_ = tx;

    // tz is a positive upward thrust magnitude; the allocator uses NED body Fz.
    force_sp_ << tx, 0.0, -tz;
    have_force_setpoint_ = true;

    if (have_attitude_setpoint_ && have_attitude_) {
      Eigen::Vector3d attitude_error = attitude_sp_ - attitude_measured_;
      for (int axis = 0; axis < 3; ++axis) {
        attitude_error(axis) = std::atan2(
          std::sin(attitude_error(axis)), std::cos(attitude_error(axis)));
      }

      // Obtain the Euler-setpoint rates from consecutive attitude setpoints.
      Eigen::Vector3d euler_rate_setpoint = Eigen::Vector3d::Zero();
      if (have_previous_attitude_setpoint_ && dt > 0.0 && std::isfinite(dt)) {
        for (int axis = 0; axis < 3; ++axis) {
          const double setpoint_delta = std::atan2(
            std::sin(attitude_sp_(axis) - previous_attitude_sp_(axis)),
            std::cos(attitude_sp_(axis) - previous_attitude_sp_(axis)));
          euler_rate_setpoint(axis) = setpoint_delta / dt;
        }
      }
      previous_attitude_sp_ = attitude_sp_;
      have_previous_attitude_setpoint_ = true;

      // P-plus-feedforward Euler rates [phi_dot_c, theta_dot_c, psi_dot_c].
      const Eigen::Vector3d euler_rate_command =
        attitude_kp_.cwiseProduct(attitude_error) + euler_rate_setpoint;

      // ZYX Euler-rate to body-rate transformation, evaluated using the
      // current measured roll and pitch on every control cycle.
      const double phi = attitude_measured_.x();
      const double theta = attitude_measured_.y();
      const double phi_dot_c = euler_rate_command.x();
      const double theta_dot_c = euler_rate_command.y();
      const double psi_dot_c = euler_rate_command.z();

      omega_sp_ <<
        phi_dot_c - std::sin(theta) * psi_dot_c,
        std::cos(phi) * theta_dot_c +
          std::sin(phi) * std::cos(theta) * psi_dot_c,
        -std::sin(phi) * theta_dot_c +
          std::cos(phi) * std::cos(theta) * psi_dot_c;

      for (int axis = 0; axis < 3; ++axis) {
        omega_sp_(axis) = std::clamp(
          omega_sp_(axis),
          -attitude_rate_limits_(axis),
          attitude_rate_limits_(axis));
      }
      have_rate_setpoint_ = true;
    }
    update_rate_controller(dt);
    publish_actuators();
  }

  void CombinedVehicleController::set_path(const int type, const double* path) {
    path_.type = type;
    gamma_ = 0.0;
    local_curvature_ = 0.0;
    tx_previous_ = 0.0;

    if (type == 0) {
      path_.waypoint << path[0], path[1], path[2];
      path_guidance_active_ = false;
      return;
    }
    if (type == 1) {
      path_.line_p0 << path[0], path[1], path[2];
      path_.line_p1 << path[3], path[4], path[5];
      path_.line_v = path[6];
    } else if (type == 2) {
      path_.circle_c << path[0], path[1], path[2];
      path_.circle_R = path[3];
      path_.circle_v = path[4];
    } else {
      path_.lemniscate_c << path[0], path[1], path[2];
      path_.lemniscate_a = path[3];
      path_.lemniscate_v = path[4];
    }

    path_guidance_active_ = true;
    have_heading_setpoint_ = false;
    have_airspeed_setpoint_ = false;
  }

  void CombinedVehicleController::update_path_guidance(
    const double dt,
    const Eigen::Vector3d& p) {
    local_curvature_ = 0.0;
    Eigen::Vector3d pd = Eigen::Vector3d::Zero();
    Eigen::Vector3d dpd_dgamma = Eigen::Vector3d::Zero();
    Eigen::Vector3d d2pd_dgamma2 = Eigen::Vector3d::Zero();
    double va = 0.0;

    if (path_.type == 1) {
      va = path_.line_v;
      pd = path_.line_p0 + gamma_ * (path_.line_p1 - path_.line_p0);
      dpd_dgamma = path_.line_p1 - path_.line_p0;
    } else if (path_.type == 2) {
      va = path_.circle_v;
      const double radius = path_.circle_R;
      const Eigen::Vector3d& center = path_.circle_c;
      pd << center.x() + radius * std::cos(gamma_),
            center.y() + radius * std::sin(gamma_),
            center.z();
      dpd_dgamma << -radius * std::sin(gamma_),
                     radius * std::cos(gamma_),
                     0.0;
      d2pd_dgamma2 << -radius * std::cos(gamma_),
                      -radius * std::sin(gamma_),
                       0.0;
    } else {
      va = path_.lemniscate_v;
      const double a = path_.lemniscate_a;
      const Eigen::Vector3d& center = path_.lemniscate_c;
      const double sine = std::sin(gamma_);
      const double cosine = std::cos(gamma_);
      const double denominator = 1.0 + sine * sine;
      pd << center.x() + a * cosine / denominator,
            center.y() + a * sine * cosine / denominator,
            center.z();
      dpd_dgamma = lemniscate_derivative(a, gamma_);
      constexpr double derivative_step = 1e-4;
      d2pd_dgamma2 =
        (lemniscate_derivative(a, gamma_ + derivative_step) -
         lemniscate_derivative(a, gamma_ - derivative_step)) /
        (2.0 * derivative_step);
    }

    const double path_derivative_norm = dpd_dgamma.norm();
    if (path_derivative_norm <= 1e-6 || !std::isfinite(path_derivative_norm)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "CombinedVehicleController received a degenerate path");
      return;
    }

    const Eigen::Vector3d position_error = p - pd;
    const Eigen::Vector3d tangent = dpd_dgamma / path_derivative_norm;
    const Eigen::Matrix3d tangent_projection =
      Eigen::Matrix3d::Identity() - tangent * tangent.transpose();
    const Eigen::Vector3d auxiliary =
      -path_k1_ * tangent_projection * position_error + path_k2_ * tangent;
    const double auxiliary_norm = auxiliary.norm();
    const double auxiliary_norm_floor = std::max(0.05 * path_k2_, 1e-3);
    if (auxiliary_norm <= auxiliary_norm_floor || !std::isfinite(auxiliary_norm)) {
      return;
    }

    heading_sp_raw_ = auxiliary / auxiliary_norm;
    heading_sp_ = heading_sp_raw_;
    position_sp_ = pd;
    airspeed_sp_ = va;
    local_curvature_ = signed_horizontal_curvature(dpd_dgamma, d2pd_dgamma2);
    have_heading_setpoint_ = true;
    have_airspeed_setpoint_ = true;

    const double gamma_dot =
      path_k1_ * va * tangent.dot(position_error) /
        (auxiliary_norm * path_derivative_norm) +
      va * path_k2_ / (auxiliary_norm * path_derivative_norm);
    if (dt > 0.0 && std::isfinite(dt) && std::isfinite(gamma_dot)) {
      gamma_ += gamma_dot * dt;
    }

    publish_plot_data(
      gamma_dot,
      va,
      pd,
      dpd_dgamma,
      {static_cast<float>(path_k1_), static_cast<float>(path_k2_)});
  }

  void CombinedVehicleController::reset() {
    rate_controller_.reset();
    omega_sp_.setZero();
    omega_measured_.setZero();
    force_sp_.setZero();
    attitude_sp_.setZero();
    previous_attitude_sp_.setZero();
    attitude_measured_.setZero();
    have_rate_setpoint_ = false;
    have_attitude_setpoint_ = false;
    have_previous_attitude_setpoint_ = false;
    have_attitude_ = false;
    have_angular_velocity_ = false;
    have_force_setpoint_ = false;
    heading_sp_raw_ = Eigen::Vector3d::UnitX();
    heading_sp_ = Eigen::Vector3d::UnitX();
    position_sp_.setZero();
    local_curvature_ = 0.0;
    tx_previous_ = 0.0;
    x_error_integral_ = 0.0;
    z_error_integral_ = 0.0;
    have_heading_setpoint_ = false;
    airspeed_sp_ = 0.0;
    have_airspeed_setpoint_ = false;
    gamma_ = 0.0;
    path_guidance_active_ = false;
    have_wrench_ = false;
  }

  void CombinedVehicleController::on_wrench_callback(
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
      "CombinedVehicleController received wrench: M=[%.3f %.3f %.3f], Fx=%.3f, Fz=%.3f, ignored Fy=%.3f",
      wrench_(0),
      wrench_(1),
      wrench_(2),
      wrench_(3),
      wrench_(4),
      msg->wrench.force.y);
  }

  void CombinedVehicleController::on_state_callback(
    const violet_msgs::msg::State::ConstSharedPtr msg) {
    attitude_measured_ <<
      msg->attitude[0], msg->attitude[1], msg->attitude[2];
    have_attitude_ = true;
  }

  void CombinedVehicleController::on_heading_setpoint_callback(
    const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
    const Eigen::Vector3d heading(msg->vector.x, msg->vector.y, msg->vector.z);
    const double norm = heading.norm();
    if (!heading.allFinite() || norm <= 1e-6) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "Ignoring invalid zero or non-finite heading setpoint");
      return;
    }

    heading_sp_raw_ = heading / norm;
    heading_sp_ = heading_sp_raw_;
    local_curvature_ = 0.0;
    have_heading_setpoint_ = true;
    path_guidance_active_ = false;
  }

  void CombinedVehicleController::on_airspeed_setpoint_callback(
    const std_msgs::msg::Float64::ConstSharedPtr msg) {
    if (!std::isfinite(msg->data) || msg->data < 0.0) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "Ignoring invalid negative or non-finite airspeed setpoint");
      return;
    }

    airspeed_sp_ = msg->data;
    have_airspeed_setpoint_ = true;
    local_curvature_ = 0.0;
    path_guidance_active_ = false;
  }

  void CombinedVehicleController::update_rate_controller(const double dt) {
    if (!have_rate_setpoint_ || !have_angular_velocity_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "CombinedVehicleController waiting for inputs: attitude_setpoint=%s attitude=%s angular_velocity=%s",
        have_attitude_setpoint_ ? "received" : "missing",
        have_attitude_ ? "received" : "missing",
        have_angular_velocity_ ? "received" : "missing");
      return;
    }

    const Eigen::Vector3d torque_sp_Nm =
      rate_controller_.update(dt, omega_sp_, omega_measured_);
    publish_wrench(torque_sp_Nm, force_sp_);

    const RateController::Debug& debug = rate_controller_.debug();
    std::ostringstream log;
    log << "CombinedVehicleController rate PID\n"
      << "  axes              roll    pitch      yaw\n"
      << "  omega_sp_rad_s    " << vector_to_string(omega_sp_) << "\n"
      << "  omega_meas_rad_s  " << vector_to_string(omega_measured_) << "\n"
      << "  error_rad_s       " << vector_to_string(debug.error) << "\n"
      << "  integral          " << vector_to_string(debug.integral) << "\n"
      << "  derivative        " << vector_to_string(debug.derivative) << "\n"
      << "  torque_raw_Nm     " << vector_to_string(debug.torque_unsaturated_Nm) << "\n"
      << "  torque_cmd_Nm     " << vector_to_string(debug.torque_saturated_Nm) << "\n"
      << "  force_sp_N        " << vector_to_string(force_sp_) << "\n"
      << "  velocity_ned_m_s  " << vector_to_string(inertial_velocity_debug_) << "\n"
      << "  u_cmd_m_s         " << u_cmd_debug_ << "\n"
      << "  u_measured_m_s    " << u_measured_debug_ << "\n"
      << "  v_lateral_m_s     " << lateral_velocity_debug_ << "\n"
      << "  wrench_cmd        " << vector_to_string(wrench_);
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, log.str());
  }

  void CombinedVehicleController::publish_wrench(
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

  Eigen::Matrix<double, 5, 5> CombinedVehicleController::build_physical_effectiveness_matrix() const {
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

  Eigen::Matrix<double, 5, 1> CombinedVehicleController::normalized_wrench_to_physical(
    const Eigen::Matrix<double, 5, 1>& u_command) const {
    (void)wrench_scale_;
    return u_command;
  }

  Eigen::Matrix<double, 5, 1> CombinedVehicleController::allocate_thrust(
    const Eigen::Matrix<double, 5, 5>& effectiveness,
    const Eigen::Matrix<double, 5, 1>& u_phys) const {
    return effectiveness.inverse() * u_phys;
  }

  Eigen::Matrix<double, 5, 1> CombinedVehicleController::thrust_to_motor_speed(
    const Eigen::Matrix<double, 5, 1>& thrust) const {
    Eigen::Matrix<double, 5, 1> omega;
    for (int i = 0; i < 5; ++i) {
      omega(i) = std::sqrt(std::max(0.0, thrust(i)) / motor_constants_(i));
    }
    return omega;
  }

  Eigen::Matrix<double, 5, 1> CombinedVehicleController::thrust_to_normalized_motor_command(
    const Eigen::Matrix<double, 5, 1>& thrust) const {
    Eigen::Matrix<double, 5, 1> motor_command;
    for (int i = 0; i < 5; ++i) {
      const double thrust_max =
        motor_constants_(i) * max_rot_velocities_(i) * max_rot_velocities_(i);
      motor_command(i) = std::sqrt(std::clamp(thrust(i), 0.0, thrust_max) / thrust_max);
    }
    return motor_command;
  }

  void CombinedVehicleController::publish_actuators() {
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
    bool negative_thrust_requested = false;
    bool non_finite_thrust_requested = false;
    for (int i = 0; i < 5; ++i) {
      const double thrust_max =
        motor_constants_(i) * max_rot_velocities_(i) * max_rot_velocities_(i);
      const double requested_thrust = thrust_before_saturation(i);
      negative_thrust_requested = negative_thrust_requested || requested_thrust < 0.0;
      non_finite_thrust_requested =
        non_finite_thrust_requested || !std::isfinite(requested_thrust);

      // These motors are non-reversible. Never forward negative or invalid
      // allocation results to PX4: explicitly request zero from that motor.
      thrust_after_saturation(i) =
        std::isfinite(requested_thrust) && requested_thrust > 0.0 ?
        std::min(requested_thrust, thrust_max) : 0.0;
      clipped = clipped || thrust_after_saturation(i) != thrust_before_saturation(i);
    }
    const Eigen::Matrix<double, 5, 1> omega_cmd = thrust_to_motor_speed(thrust_after_saturation);
    const Eigen::Matrix<double, 5, 1> actuator_commands =
      thrust_to_normalized_motor_command(thrust_after_saturation);
    const Eigen::Matrix<double, 5, 1> achieved_wrench =
      effectiveness * (thrust_after_saturation - thrust_trim_);
    const Eigen::Matrix<double, 5, 1> wrench_error = u_phys - achieved_wrench;

    geometry_msgs::msg::WrenchStamped achieved_wrench_msg;
    achieved_wrench_msg.header.stamp = node_->get_clock()->now();
    achieved_wrench_msg.header.frame_id = "combined_body";
    achieved_wrench_msg.wrench.torque.x = achieved_wrench(0);
    achieved_wrench_msg.wrench.torque.y = achieved_wrench(1);
    achieved_wrench_msg.wrench.torque.z = achieved_wrench(2);
    achieved_wrench_msg.wrench.force.x = achieved_wrench(3);
    achieved_wrench_msg.wrench.force.y = 0.0;
    achieved_wrench_msg.wrench.force.z = achieved_wrench(4);
    achieved_wrench_pub_->publish(achieved_wrench_msg);

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
    allocation_log << "CombinedVehicleController physical allocation" << (clipped ? " clipped" : "") << "\n";
    if (negative_thrust_requested) {
      allocation_log
        << "  note: negative motor thrust request replaced with a 0 command\n";
    }
    if (non_finite_thrust_requested) {
      allocation_log
        << "  note: non-finite motor thrust request replaced with a 0 command\n";
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
      "CombinedVehicleController actuator commands: q=[%.3f %.3f %.3f %.3f], prop=%.3f, surf=[%.3f %.3f %.3f %.3f]",
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

  void CombinedVehicleController::publish_offboard_heartbeat() {
    publish_offboard_mode();
    publish_offboard_request(shuttle_mode_pub_, vehicle_id_);
    publish_offboard_request(fixed_wing_mode_pub_, fixed_wing_vehicle_id_);
  }

  void CombinedVehicleController::publish_offboard_mode() {
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

  void CombinedVehicleController::publish_offboard_request(
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
