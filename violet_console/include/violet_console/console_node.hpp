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

    // UI Object
    ConsoleUI::UniquePtr console_ui_;

  protected:
    std::string vehicle_ns_;

    // ROS2 thread setup
    std::thread executor_thread_;
    rclcpp::executors::MultiThreadedExecutor executor_;
};