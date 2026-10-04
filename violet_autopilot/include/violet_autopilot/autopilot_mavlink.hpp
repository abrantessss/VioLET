#pragma once

#include <chrono>
#include <mutex>
#include <mavsdk/mavsdk.h>
#include <mavsdk/system.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "mellinger_controller_mavlink.hpp"
#include "los4_controller_mavlink.hpp"
#include "violet_msgs/msg/mode.hpp"
#include "violet_msgs/msg/trajectory.hpp"
#include "violet_msgs/msg/state.hpp"
#include "violet_msgs/msg/status.hpp"
#include "violet_msgs/msg/battery.hpp"
#include "violet_msgs/srv/mode.hpp"

// One SDK connection owns both telemetry and commands; no PX4 ROS topics.
class AutopilotMavlink : public rclcpp::Node {
public:
  AutopilotMavlink();
  ~AutopilotMavlink();
  void start();
  void update();
private:
  using Clock = std::chrono::steady_clock;
  struct TelemetryCache {
    std::mutex mutex;
    violet_msgs::msg::State state;
    Clock::time_point position_time{}, attitude_time{};
  };
  bool read_state(violet_msgs::msg::State& state);
  void follow(const violet_msgs::msg::Trajectory& trajectory);
  bool command(const std::string& mode);
  void stop_follow();

  // Callbacks capture the cache, never this, so shutdown cannot access a dead node.
  std::shared_ptr<TelemetryCache> cache_{std::make_shared<TelemetryCache>()};
  std::unique_ptr<mavsdk::Mavsdk> sdk_;
  std::shared_ptr<mavsdk::System> system_;
  std::unique_ptr<mavsdk::Action> action_;
  std::unique_ptr<mavsdk::Telemetry> telemetry_;
  std::shared_ptr<mavsdk::Offboard> offboard_;
  autopilot::Controller::UniquePtr controller_;
  std::vector<rclcpp::Subscription<violet_msgs::msg::Mode>::SharedPtr> mode_subs_;
  std::vector<rclcpp::Service<violet_msgs::srv::Mode>::SharedPtr> mode_services_;
  rclcpp::Subscription<violet_msgs::msg::Trajectory>::SharedPtr follow_sub_;
  rclcpp::Publisher<violet_msgs::msg::State>::SharedPtr state_pub_;
  rclcpp::Publisher<violet_msgs::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<violet_msgs::msg::Battery>::SharedPtr battery_pub_;
  int vehicle_id_{1};
  double update_rate_{100.0}, telemetry_timeout_{0.5};
  bool following_{false}, offboard_started_{false};
  Clock::time_point prime_time_{}, previous_time_{}, status_time_{}, started_time_{};
};
