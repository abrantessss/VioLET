#pragma once

#include <thread>
#include <memory>
#include "console_ui.hpp"
#include "rclcpp/rclcpp.hpp"

// ROS2 messages
#include "violet_msgs/msg/battery.hpp"
#include "violet_msgs/msg/state.hpp"
#include "violet_msgs/msg/status.hpp"

class ConsoleNode : public rclcpp::Node {

  public:
    ConsoleNode(const std::string vehicle_ns, const unsigned int vehicle_id);
    ~ConsoleNode();

    void start();

    // ROS2 pub, subs and services
    void init_subscribers();

    // Control Functions
    void on_arm_click();
    void on_disarm_click();
    void on_land_click();
    void on_orbit_click();
    void on_takeoff_click();
    void on_kill_click();

    // Trajectory Functions
    void on_waypoint_click();
    void on_line_click();
    void on_circle_click();
    void on_lemniscate_click();

    // UI Object
    ConsoleUI::UniquePtr console_ui_;

  protected:
    std::string vehicle_ns_;
    int vehicle_id_;

    // Config console UI
    ConsoleUI::Config config_;

    // ROS2 thread setup
    std::thread executor_thread_;
    rclcpp::executors::MultiThreadedExecutor executor_;

    // ROS2 subscribers
    rclcpp::Subscription<violet_msgs::msg::Battery>::SharedPtr battery_sub_;
    rclcpp::Subscription<violet_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<violet_msgs::msg::Status>::SharedPtr status_sub_;

    // Callbacks for ROS2 subscribers
    void on_battery_callback(const violet_msgs::msg::Battery::ConstSharedPtr msg);
    void on_state_callback(const violet_msgs::msg::State::ConstSharedPtr msg);
    void on_status_callback(const violet_msgs::msg::Status::ConstSharedPtr msg);
};