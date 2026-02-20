#include "console_node.hpp"
#include <thread>
#include <chrono>


ConsoleNode::ConsoleNode(const std::string vehicle_ns, const unsigned int vehicle_id) : rclcpp::Node("violet_console") {
  
  // Initialize vehicle namespace
  vehicle_ns_ = std::string(vehicle_ns + std::to_string(vehicle_id));

  // Initialize the callbacks for the console UI
  config_.on_arm_click = std::bind(&ConsoleNode::on_arm_click, this);
  config_.on_disarm_click = std::bind(&ConsoleNode::on_disarm_click, this);
  config_.on_land_click = std::bind(&ConsoleNode::on_land_click, this);
  config_.on_hold_click = std::bind(&ConsoleNode::on_hold_click, this);
  config_.on_takeoff_click = std::bind(&ConsoleNode::on_takeoff_click, this);
  config_.on_kill_click = std::bind(&ConsoleNode::on_kill_click, this);

  // Add trajectory follow
  config_.on_waypoint_click = std::bind(&ConsoleNode::on_waypoint_click, this);
  config_.on_line_click = std::bind(&ConsoleNode::on_line_click, this);
  config_.on_circle_click = std::bind(&ConsoleNode::on_circle_click, this);
  config_.on_lemniscate_click = std::bind(&ConsoleNode::on_lemniscate_click, this);

  // Initialize the console UI
  console_ui_ = std::make_unique<ConsoleUI>(config_);
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

void ConsoleNode::on_arm_click() {
  console_ui_->telemetry_data_.status.mode = 0;
}

void ConsoleNode::on_disarm_click() {
  console_ui_->telemetry_data_.status.mode = 1;
}

void ConsoleNode::on_takeoff_click() {
  console_ui_->telemetry_data_.status.mode = 2;
}

void ConsoleNode::on_hold_click() {
  console_ui_->telemetry_data_.status.mode = 3;
}

void ConsoleNode::on_land_click() {
  console_ui_->telemetry_data_.status.mode = 4;
}

void ConsoleNode::on_kill_click() {
  console_ui_->telemetry_data_.status.mode = 5;
}

void ConsoleNode::on_waypoint_click() {
  printf("%s %s %s\n",
       console_ui_->trajectory_data_.waypoint_pos_input[0].c_str(),
       console_ui_->trajectory_data_.waypoint_pos_input[1].c_str(),
       console_ui_->trajectory_data_.waypoint_pos_input[2].c_str());

  std::this_thread::sleep_for(std::chrono::seconds(1));

}

void ConsoleNode::on_line_click() {
  printf("%s %s %s %s %s %s %s\n",
       console_ui_->trajectory_data_.line_pos_input[0].c_str(),
       console_ui_->trajectory_data_.line_pos_input[1].c_str(),
       console_ui_->trajectory_data_.line_pos_input[2].c_str(),
       console_ui_->trajectory_data_.line_pos_input[3].c_str(),
       console_ui_->trajectory_data_.line_pos_input[4].c_str(),
       console_ui_->trajectory_data_.line_pos_input[5].c_str(),
       console_ui_->trajectory_data_.line_speed_input.c_str());

  std::this_thread::sleep_for(std::chrono::seconds(1));
}

void ConsoleNode::on_circle_click() {
  printf("%s %s %s %s %s\n",
       console_ui_->trajectory_data_.circle_pos_input[0].c_str(),
       console_ui_->trajectory_data_.circle_pos_input[1].c_str(),
       console_ui_->trajectory_data_.circle_pos_input[2].c_str(),
       console_ui_->trajectory_data_.circle_pos_input[3].c_str(),
       console_ui_->trajectory_data_.circle_speed_input.c_str());

  std::this_thread::sleep_for(std::chrono::seconds(1));
}

void ConsoleNode::on_lemniscate_click() {
  printf("%s %s %s %s %s\n",
       console_ui_->trajectory_data_.lemniscate_pos_input[0].c_str(),
       console_ui_->trajectory_data_.lemniscate_pos_input[1].c_str(),
       console_ui_->trajectory_data_.lemniscate_pos_input[2].c_str(),
       console_ui_->trajectory_data_.lemniscate_pos_input[3].c_str(),
       console_ui_->trajectory_data_.lemniscate_speed_input.c_str());

  std::this_thread::sleep_for(std::chrono::seconds(1));
}