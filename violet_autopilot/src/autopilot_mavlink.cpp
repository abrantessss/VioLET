#include "autopilot_mavlink.hpp"

#include <cmath>
#include <algorithm>
#include <thread>
#include <stdexcept>

AutopilotMavlink::AutopilotMavlink() : Node("autopilot_node_mavlink") {}

AutopilotMavlink::~AutopilotMavlink() {
  if (following_ && offboard_) stop_follow();
}

void AutopilotMavlink::start() {
  vehicle_id_ = declare_parameter<int>("vehicle_id", 1);
  const auto type = declare_parameter<std::string>("controllers.type", "mellinger_mavlink");
  if (type != "mellinger_mavlink" && type != "los4_mavlink") {
    throw std::invalid_argument("MAVLink autopilot supports mellinger_mavlink or los4_mavlink");
  }
  update_rate_ = declare_parameter<double>("controllers.update_rate_hz", 100.0);
  telemetry_timeout_ = declare_parameter<double>("mavlink.telemetry_timeout_s", 0.5);
  const auto timeout = declare_parameter<double>("mavlink.discovery_timeout_s", 30.0);
  const auto telemetry_rate = declare_parameter<double>("mavlink.telemetry_rate_hz", 100.0);
  const auto url = declare_parameter<std::string>("mavlink.connection_url", "udp://:14540");
  for (double value : {update_rate_, telemetry_timeout_, timeout, telemetry_rate}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("MAVLink rates and timeouts must be positive and finite");
    }
  }
  if (vehicle_id_ < 1 || vehicle_id_ > 255) throw std::invalid_argument("Invalid vehicle_id");

  sdk_ = std::make_unique<mavsdk::Mavsdk>(
    mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::CompanionComputer});
  if (sdk_->add_any_connection(url) != mavsdk::ConnectionResult::Success) {
    throw std::runtime_error("Cannot open MAVLink connection: " + url);
  }
  RCLCPP_INFO(get_logger(), "Waiting for MAVLink system %d on %s", vehicle_id_, url.c_str());
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout);
  while (rclcpp::ok() && Clock::now() < deadline && !system_) {
    for (const auto& candidate : sdk_->systems()) {
      if (candidate->is_connected() && candidate->has_autopilot() &&
          candidate->get_system_id() == vehicle_id_) {
        system_ = candidate;
        break;
      }
    }
    if (!system_) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!system_) throw std::runtime_error("Timed out discovering requested MAVLink system");
  action_ = std::make_unique<mavsdk::Action>(system_);
  telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
  offboard_ = std::make_shared<mavsdk::Offboard>(system_);

  const auto cache = cache_;
  telemetry_->subscribe_position_velocity_ned([cache](mavsdk::Telemetry::PositionVelocityNed value) {
    std::lock_guard<std::mutex> lock(cache->mutex);
    cache->state.position = {value.position.north_m, value.position.east_m, value.position.down_m};
    cache->state.inertial_velocity = {
      value.velocity.north_m_s, value.velocity.east_m_s, value.velocity.down_m_s};
    cache->position_time = Clock::now();
  });
  // Use the quaternion stream directly: MAVSDK 2.0.1's Euler callback
  // reads cached quaternion data even when an ATTITUDE packet triggers it.
  telemetry_->subscribe_attitude_quaternion([cache](mavsdk::Telemetry::Quaternion value) {
    Eigen::Quaterniond q(value.w, value.x, value.y, value.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-6) return;
    q.normalize();
    std::lock_guard<std::mutex> lock(cache->mutex);
    cache->state.attitude = {
      static_cast<float>(std::atan2(2.0 * (q.w()*q.x() + q.y()*q.z()),
                                   1.0 - 2.0 * (q.x()*q.x() + q.y()*q.y()))),
      static_cast<float>(std::asin(std::clamp(2.0 * (q.w()*q.y() - q.z()*q.x()), -1.0, 1.0))),
      static_cast<float>(std::atan2(2.0 * (q.w()*q.z() + q.x()*q.y()),
                                   1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z())))};
    cache->attitude_time = Clock::now();
  });
  telemetry_->subscribe_attitude_angular_velocity_body([cache](mavsdk::Telemetry::AngularVelocityBody value) {
    std::lock_guard<std::mutex> lock(cache->mutex);
    cache->state.angular_velocity = {value.roll_rad_s, value.pitch_rad_s, value.yaw_rad_s};
  });
  for (const auto result : {
      telemetry_->set_rate_position_velocity_ned(telemetry_rate),
      telemetry_->set_rate_attitude_quaternion(telemetry_rate)}) {
    if (result != mavsdk::Telemetry::Result::Success) {
      RCLCPP_WARN_STREAM(get_logger(), "Telemetry rate request failed: " << result);
    }
  }

  state_pub_ = create_publisher<violet_msgs::msg::State>("fmu/telemetry/state", rclcpp::SensorDataQoS());
  status_pub_ = create_publisher<violet_msgs::msg::Status>("fmu/telemetry/status", rclcpp::SensorDataQoS());
  battery_pub_ = create_publisher<violet_msgs::msg::Battery>("fmu/telemetry/battery", rclcpp::SensorDataQoS());
  if (type == "los4_mavlink") {
    controller_ = std::make_unique<autopilot::LOS4ControllerMavlink>(offboard_);
  } else {
    controller_ = std::make_unique<autopilot::MellingerControllerMavlink>(offboard_);
  }
  // Non-owning node handle avoids the node/controller shared_ptr ownership cycle.
  controller_->initialize_controller({rclcpp::Node::SharedPtr(this, [](rclcpp::Node*) {}), vehicle_id_});

  for (const std::string mode : {"arm", "disarm", "takeoff", "loiter", "land", "kill"}) {
    const auto topic = declare_parameter<std::string>("subscribers.mode." + mode, "fmu/mode/" + mode);
    mode_subs_.push_back(create_subscription<violet_msgs::msg::Mode>(
      topic, rclcpp::SensorDataQoS(),
      [this, mode](violet_msgs::msg::Mode::ConstSharedPtr) { command(mode); }));
    const auto service = declare_parameter<std::string>("services.mode." + mode, "fmu/mode/" + mode);
    mode_services_.push_back(create_service<violet_msgs::srv::Mode>(service,
      [this, mode](const violet_msgs::srv::Mode::Request::SharedPtr,
                   const violet_msgs::srv::Mode::Response::SharedPtr response) {
        response->success = command(mode);
      }));
  }
  follow_sub_ = create_subscription<violet_msgs::msg::Trajectory>(
    declare_parameter<std::string>("subscribers.mode.follow", "fmu/mode/follow"), rclcpp::SensorDataQoS(),
    [this](violet_msgs::msg::Trajectory::ConstSharedPtr msg) { follow(*msg); });
  RCLCPP_INFO(get_logger(), "MAVLink autopilot ready: %s", type.c_str());
}

bool AutopilotMavlink::read_state(violet_msgs::msg::State& state) {
  std::lock_guard<std::mutex> lock(cache_->mutex);
  state = cache_->state;
  const auto now = Clock::now();
  if (std::chrono::duration<double>(now - cache_->position_time).count() > telemetry_timeout_ ||
      std::chrono::duration<double>(now - cache_->attitude_time).count() > telemetry_timeout_) return false;
  for (const auto& values : {state.position, state.inertial_velocity, state.attitude}) {
    for (float value : values) if (!std::isfinite(value)) return false;
  }
  return system_->is_connected();
}

void AutopilotMavlink::follow(const violet_msgs::msg::Trajectory& trajectory) {
  violet_msgs::msg::State state;
  if (!read_state(state) || !telemetry_->armed()) {
    RCLCPP_WARN(get_logger(), "FOLLOW requires an armed vehicle and fresh finite NED position/attitude");
    return;
  }
  std::vector<double> path;
  switch (trajectory.path_type) {
    case 0: path.assign(trajectory.waypoint.begin(), trajectory.waypoint.end()); break;
    case 1: path.assign(trajectory.line.begin(), trajectory.line.end()); break;
    case 2: path.assign(trajectory.circle.begin(), trajectory.circle.end()); break;
    case 3: path.assign(trajectory.lemniscate.begin(), trajectory.lemniscate.end()); break;
    default: RCLCPP_WARN(get_logger(), "Unknown trajectory type"); return;
  }
  for (double value : path) {
    if (!std::isfinite(value)) { RCLCPP_WARN(get_logger(), "Non-finite trajectory"); return; }
  }
  try {
    controller_->set_path(trajectory.path_type, path.data());
  } catch (const std::invalid_argument& error) {
    RCLCPP_WARN(get_logger(), "FOLLOW rejected: %s", error.what());
    return;
  }
  if (!following_) {
    prime_time_ = Clock::now();
    offboard_started_ = false;
  }
  previous_time_ = Clock::now();
  following_ = true;
}

void AutopilotMavlink::stop_follow() {
  following_ = false;
  offboard_started_ = false;
  // stop() also cancels MAVSDK's automatic retransmission of the last setpoint.
  const auto result = offboard_->stop();
  if (result != mavsdk::Offboard::Result::Success) {
    RCLCPP_WARN_STREAM(get_logger(), "Offboard stop/Hold request failed: " << result);
  }
}

bool AutopilotMavlink::command(const std::string& mode) {
  // Emergency commands go first, without waiting for a Hold acknowledgement.
  if (mode == "kill" || mode == "disarm") {
    const auto result = mode == "kill" ? action_->kill() : action_->disarm();
    if (following_) stop_follow();
    RCLCPP_INFO_STREAM(get_logger(), mode << ": " << result);
    return result == mavsdk::Action::Result::Success;
  }
  if (following_) stop_follow();
  mavsdk::Action::Result result;
  if (mode == "arm") result = action_->arm();
  else if (mode == "takeoff") result = action_->takeoff();
  else if (mode == "loiter") result = action_->hold();
  else result = action_->land();
  RCLCPP_INFO_STREAM(get_logger(), mode << ": " << result);
  return result == mavsdk::Action::Result::Success;
}

void AutopilotMavlink::update() {
  rclcpp::WallRate rate(update_rate_);
  while (rclcpp::ok()) {
    rclcpp::spin_some(get_node_base_interface());
    const auto now = Clock::now();
    violet_msgs::msg::State state;
    const bool fresh = read_state(state);
    if (fresh) {
      state.header.stamp = get_clock()->now();
      state_pub_->publish(state);
    }
    if (following_) {
      if (!fresh || !telemetry_->armed() ||
          (offboard_started_ && std::chrono::duration<double>(now - started_time_).count() > 2.0 &&
           telemetry_->flight_mode() != mavsdk::Telemetry::FlightMode::Offboard)) {
        RCLCPP_ERROR(get_logger(), "Stopping FOLLOW: telemetry lost, disarmed, or Offboard exited");
        stop_follow();
      } else {
        const double dt = std::chrono::duration<double>(now - previous_time_).count();
        previous_time_ = now;
        try {
          controller_->set_attitude_rate(dt,
            Eigen::Vector3d(state.position[0], state.position[1], state.position[2]),
            Eigen::Vector3d(state.inertial_velocity[0], state.inertial_velocity[1], state.inertial_velocity[2]),
            Eigen::Vector3d(state.attitude[0], state.attitude[1], state.attitude[2]));
          if (!offboard_started_ && std::chrono::duration<double>(now - prime_time_).count() >= 1.0) {
            const auto result = offboard_->start();
            if (result != mavsdk::Offboard::Result::Success) {
              RCLCPP_ERROR_STREAM(get_logger(), "Offboard start failed: " << result);
              stop_follow();
            } else {
              offboard_started_ = true;
              started_time_ = previous_time_ = Clock::now();
            }
          }
        } catch (const std::exception& error) {
          RCLCPP_ERROR(get_logger(), "%s", error.what());
          stop_follow();
        }
      }
    }
    if (system_->is_connected() && std::chrono::duration<double>(now - status_time_).count() >= 0.2) {
      status_time_ = now;
      violet_msgs::msg::Status status;
      status.header.stamp = get_clock()->now();
      status.id = vehicle_id_;
      status.armed = telemetry_->armed();
      // Match the existing interface/console wire values (the .msg constants differ).
      switch (telemetry_->flight_mode()) {
        case mavsdk::Telemetry::FlightMode::Takeoff: status.mode = 0; break;
        case mavsdk::Telemetry::FlightMode::Land: status.mode = 1; break;
        case mavsdk::Telemetry::FlightMode::Hold: status.mode = 2; break;
        case mavsdk::Telemetry::FlightMode::Offboard: status.mode = 3; break;
        default: status.mode = 255;
      }
      status_pub_->publish(status);
      const auto value = telemetry_->battery();
      violet_msgs::msg::Battery battery;
      battery.header.stamp = status.header.stamp;
      battery.soc = value.remaining_percent;
      battery.voltage = value.voltage_v;
      battery.current = value.current_battery_a;
      battery_pub_->publish(battery);
    }
    rate.sleep();
  }
  if (following_) stop_follow();
}
