#include "console_node.hpp"
#include <thread>
#include <chrono>


ConsoleNode::ConsoleNode(const std::string vehicle_ns, const unsigned int vehicle_id) : rclcpp::Node("violet_console") {
  
  // Initialize vehicle namespace
  vehicle_ns_ = std::string(vehicle_ns + std::to_string(vehicle_id));
  vehicle_id_ = vehicle_id;

  // Initialize the callbacks for the console UI
  config_.on_arm_click = std::bind(&ConsoleNode::on_arm_click, this);
  config_.on_disarm_click = std::bind(&ConsoleNode::on_disarm_click, this);
  config_.on_land_click = std::bind(&ConsoleNode::on_land_click, this);
  config_.on_orbit_click = std::bind(&ConsoleNode::on_orbit_click, this);
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
  // Initialise pubs, subs and services
  init_subscribers();

  // Add this node to the multithread executor
  executor_.add_node(this->shared_from_this());

  // Start the executor in a separate thread
  executor_thread_ = std::thread([this]() {this->executor_.spin();});

  // Start the console UI in this thread
  console_ui_->loop();
}

void ConsoleNode::init_subscribers() {
  /**
   * Subscribe to battery status
   */
  this->declare_parameter<std::string>("subscribers.telemetry.battery", vehicle_ns_ + std::string("/fmu/telemetry/battery"));
  rclcpp::Parameter battery_topic = this->get_parameter("subscribers.telemetry.battery");
  battery_sub_ = this->create_subscription<violet_msgs::msg::Battery>(
    battery_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ConsoleNode::on_battery_callback, this, std::placeholders::_1));
  /**
   * Subscribe to state
   */
  this->declare_parameter<std::string>("subscribers.telemetry.state", vehicle_ns_ + std::string("/fmu/telemetry/state"));
  rclcpp::Parameter state_topic = this->get_parameter("subscribers.telemetry.state");
  state_sub_ = this->create_subscription<violet_msgs::msg::State>(
    state_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ConsoleNode::on_state_callback, this, std::placeholders::_1));
  /**
   * Subscribe to vehicle status
   */
  this->declare_parameter<std::string>("subscribers.telemetry.status", vehicle_ns_ + std::string("/fmu/telemetry/status"));
  rclcpp::Parameter status_topic = this->get_parameter("subscribers.telemetry.status");
  status_sub_ = this->create_subscription<violet_msgs::msg::Status>(
    status_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ConsoleNode::on_status_callback, this, std::placeholders::_1));
}

void ConsoleNode::on_arm_click() {
  console_ui_->telemetry_data_.status.mode = 0;
}

void ConsoleNode::on_disarm_click() {
  console_ui_->telemetry_data_.status.mode = 0;
}

void ConsoleNode::on_takeoff_click() {
  console_ui_->telemetry_data_.status.mode = 0;
}

void ConsoleNode::on_orbit_click() {
  console_ui_->telemetry_data_.status.mode = 0;
}

void ConsoleNode::on_land_click() {
  console_ui_->telemetry_data_.status.mode = 0;
}

void ConsoleNode::on_kill_click() {
  console_ui_->telemetry_data_.status.mode = 0;
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

void ConsoleNode::on_battery_callback(const violet_msgs::msg::Battery::ConstSharedPtr msg) {
  console_ui_->telemetry_data_.battery.soc = msg->soc;
  console_ui_->telemetry_data_.battery.voltage = msg->voltage;
  console_ui_->telemetry_data_.battery.current = msg->current;
  console_ui_->request_refresh();
}

void ConsoleNode::on_state_callback(const violet_msgs::msg::State::ConstSharedPtr msg) {
  console_ui_->telemetry_data_.state.position[0] = msg->position[0];
  console_ui_->telemetry_data_.state.position[1] = msg->position[1];
  console_ui_->telemetry_data_.state.position[2] = msg->position[2];

  console_ui_->telemetry_data_.state.attitude[0] = msg->attitude[0];
  console_ui_->telemetry_data_.state.attitude[1] = msg->attitude[1];
  console_ui_->telemetry_data_.state.attitude[2] = msg->attitude[2];

  console_ui_->telemetry_data_.state.inertial_velocity[0] = msg->inertial_velocity[0];
  console_ui_->telemetry_data_.state.inertial_velocity[1] = msg->inertial_velocity[1];
  console_ui_->telemetry_data_.state.inertial_velocity[2] = msg->inertial_velocity[2];

  console_ui_->telemetry_data_.state.angular_velocity[0] = msg->angular_velocity[0];
  console_ui_->telemetry_data_.state.angular_velocity[1] = msg->angular_velocity[1];
  console_ui_->telemetry_data_.state.angular_velocity[2] = msg->angular_velocity[2];
  
  console_ui_->request_refresh();
}

void ConsoleNode::on_status_callback(const violet_msgs::msg::Status::ConstSharedPtr msg) {
  console_ui_->telemetry_data_.status.vehicle_id = msg->id;
  console_ui_->telemetry_data_.status.armed = msg->armed;
  console_ui_->telemetry_data_.status.mode = msg->mode;
  console_ui_->request_refresh();
}

