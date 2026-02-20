#include "console_node.hpp"

ConsoleNode::ConsoleNode(const std::string vehicle_ns, const unsigned int vehicle_id) : rclcpp::Node("violet_console") {
  
  // Initialize vehicle namespace
  vehicle_ns_ = std::string(vehicle_ns + std::to_string(vehicle_id));

  // Initialize the console UI
  console_ui_ = std::make_unique<ConsoleUI>();
}

ConsoleNode::~ConsoleNode() {}

void ConsoleNode::start() {
  // Add this node to the multithread executor
  executor_.add_node(this->shared_from_this());

  // Start the executor in a separate thread
  executor_thread_ = std::thread([this]() {this->executor_.spin();});

  // Start the console UI in this thread
  console_ui_->loop();
}