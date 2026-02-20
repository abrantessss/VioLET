#pragma once

#include <thread>
#include <memory>
#include "console_ui.hpp"
#include "rclcpp/rclcpp.hpp"

class ConsoleNode : public rclcpp::Node {

  public:
    ConsoleNode(const std::string vehicle_ns, const unsigned int vehicle_id);
    ~ConsoleNode();

    void start();

    // Control Functions
    void on_arm_click();
    void on_disarm_click();
    void on_land_click();
    void on_hold_click();
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

    // Config console UI
    ConsoleUI::Config config_;

    // ROS2 thread setup
    std::thread executor_thread_;
    rclcpp::executors::MultiThreadedExecutor executor_;
};