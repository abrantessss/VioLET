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
  config_.on_loiter_click = std::bind(&ConsoleNode::on_loiter_click, this);
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
  init_publishers();

  // Add this node to the multithread executor
  executor_.add_node(this->shared_from_this());

  // Start the executor in a separate thread
  executor_thread_ = std::thread([this]() {this->executor_.spin();});

  // Start the console UI in this thread
  console_ui_->loop();
}

void ConsoleNode::init_publishers() {
  /**
   * Publisher to arm the vehicle
   */
  this->declare_parameter<std::string>("publishers.mode.arm", vehicle_ns_ +  std::string("/fmu/mode/arm"));
  rclcpp::Parameter arm_topic = this->get_parameter("publishers.mode.arm");
  arm_pub_ = this->create_publisher<violet_msgs::msg::Mode>
    (arm_topic.as_string(), rclcpp::SensorDataQoS());
  /**
   * Publisher to disarm the vehicle
   */
  this->declare_parameter<std::string>("publishers.mode.disarm", vehicle_ns_ +  std::string("/fmu/mode/disarm"));
  rclcpp::Parameter disarm_topic = this->get_parameter("publishers.mode.disarm");
  disarm_pub_ = this->create_publisher<violet_msgs::msg::Mode>
    (disarm_topic.as_string(), rclcpp::SensorDataQoS());
  /**
   * Publisher to takeoff the vehicle
   */
  this->declare_parameter<std::string>("publishers.mode.takeoff", vehicle_ns_ +  std::string("/fmu/mode/takeoff"));
  rclcpp::Parameter takeoff_topic = this->get_parameter("publishers.mode.takeoff");
  takeoff_pub_ = this->create_publisher<violet_msgs::msg::Mode>
    (takeoff_topic.as_string(), rclcpp::SensorDataQoS());
  /**
   * Publisher to loiter the vehicle
   */
  this->declare_parameter<std::string>("publishers.mode.loiter", vehicle_ns_ +  std::string("/fmu/mode/loiter"));
  rclcpp::Parameter loiter_topic = this->get_parameter("publishers.mode.loiter");
  loiter_pub_ = this->create_publisher<violet_msgs::msg::Mode>
    (loiter_topic.as_string(), rclcpp::SensorDataQoS());
  /**
   * Publisher to land the vehicle
   */
  this->declare_parameter<std::string>("publishers.mode.land", vehicle_ns_ +  std::string("/fmu/mode/land"));
  rclcpp::Parameter land_topic = this->get_parameter("publishers.mode.land");
  land_pub_ = this->create_publisher<violet_msgs::msg::Mode>
    (land_topic.as_string(), rclcpp::SensorDataQoS());
  /**
   * Publisher to kill the vehicle
   */
  this->declare_parameter<std::string>("publishers.mode.kill", vehicle_ns_ +  std::string("/fmu/mode/kill"));
  rclcpp::Parameter kill_topic = this->get_parameter("publishers.mode.kill");
  kill_pub_ = this->create_publisher<violet_msgs::msg::Mode>
    (kill_topic.as_string(), rclcpp::SensorDataQoS());
  /**
   * Publisher to follow mode
   */
  this->declare_parameter<std::string>("publishers.mode.follow", vehicle_ns_ +  std::string("/fmu/mode/follow"));
  rclcpp::Parameter follow_topic = this->get_parameter("publishers.mode.follow");
  follow_pub_ = this->create_publisher<violet_msgs::msg::Trajectory>
    (follow_topic.as_string(), rclcpp::SensorDataQoS());
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
  violet_msgs::msg::Mode arm_msg;
  arm_msg.mode = true;
  arm_pub_->publish(arm_msg);
}

void ConsoleNode::on_disarm_click() {
  violet_msgs::msg::Mode disarm_msg;
  disarm_msg.mode = true;
  disarm_pub_->publish(disarm_msg);
}

void ConsoleNode::on_takeoff_click() {
  violet_msgs::msg::Mode takeoff_msg;
  takeoff_msg.mode = true;
  takeoff_pub_->publish(takeoff_msg);
}

void ConsoleNode::on_loiter_click() {
  violet_msgs::msg::Mode loiter_msg;
  loiter_msg.mode = true;
  loiter_pub_->publish(loiter_msg);
}

void ConsoleNode::on_land_click() {
  violet_msgs::msg::Mode land_msg;
  land_msg.mode = true;
  land_pub_->publish(land_msg);
}

void ConsoleNode::on_kill_click() {
  violet_msgs::msg::Mode kill_msg;
  kill_msg.mode = true;
  kill_pub_->publish(kill_msg);
}

void ConsoleNode::on_waypoint_click() {
  if (!console_ui_->trajectory_data_.waypoint_pos_input[0].empty())
    console_ui_->trajectory_data_.waypoint[0] = std::stod(console_ui_->trajectory_data_.waypoint_pos_input[0]);
  if (!console_ui_->trajectory_data_.waypoint_pos_input[1].empty())
    console_ui_->trajectory_data_.waypoint[1] = std::stod(console_ui_->trajectory_data_.waypoint_pos_input[1]);
  if (!console_ui_->trajectory_data_.waypoint_pos_input[2].empty())
    console_ui_->trajectory_data_.waypoint[2] = std::stod(console_ui_->trajectory_data_.waypoint_pos_input[2]);
  
  violet_msgs::msg::Trajectory follow_msg;
  follow_msg.path_type = 0; // Waypoint
  follow_msg.waypoint[0] = console_ui_->trajectory_data_.waypoint[0]; 
  follow_msg.waypoint[1] = console_ui_->trajectory_data_.waypoint[1];
  follow_msg.waypoint[2] = console_ui_->trajectory_data_.waypoint[2];
  follow_pub_->publish(follow_msg);
}


void ConsoleNode::on_line_click() {
  if (!console_ui_->trajectory_data_.line_pos_input[0].empty())
    console_ui_->trajectory_data_.line[0] = std::stod(console_ui_->trajectory_data_.line_pos_input[0]);
  if (!console_ui_->trajectory_data_.line_pos_input[1].empty())
    console_ui_->trajectory_data_.line[1] = std::stod(console_ui_->trajectory_data_.line_pos_input[1]);
  if (!console_ui_->trajectory_data_.line_pos_input[2].empty())
    console_ui_->trajectory_data_.line[2] = std::stod(console_ui_->trajectory_data_.line_pos_input[2]);
  if (!console_ui_->trajectory_data_.line_pos_input[3].empty())
    console_ui_->trajectory_data_.line[3] = std::stod(console_ui_->trajectory_data_.line_pos_input[3]);
  if (!console_ui_->trajectory_data_.line_pos_input[4].empty())
    console_ui_->trajectory_data_.line[4] = std::stod(console_ui_->trajectory_data_.line_pos_input[4]);
  if (!console_ui_->trajectory_data_.line_pos_input[5].empty())
    console_ui_->trajectory_data_.line[5] = std::stod(console_ui_->trajectory_data_.line_pos_input[5]);
  if (!console_ui_->trajectory_data_.line_speed_input.empty())
    console_ui_->trajectory_data_.line[6] = std::stod(console_ui_->trajectory_data_.line_speed_input);
  
  violet_msgs::msg::Trajectory follow_msg;
  follow_msg.path_type = 1; // Line
  follow_msg.line[0] = console_ui_->trajectory_data_.line[0];
  follow_msg.line[1] = console_ui_->trajectory_data_.line[1];  
  follow_msg.line[2] = console_ui_->trajectory_data_.line[2];
  follow_msg.line[3] = console_ui_->trajectory_data_.line[3];
  follow_msg.line[4] = console_ui_->trajectory_data_.line[4];
  follow_msg.line[5] = console_ui_->trajectory_data_.line[5];
  follow_msg.line[6] = console_ui_->trajectory_data_.line[6];
  follow_pub_->publish(follow_msg);
}

void ConsoleNode::on_circle_click() {
  if (!console_ui_->trajectory_data_.circle_pos_input[0].empty())
    console_ui_->trajectory_data_.circle[0] = std::stod(console_ui_->trajectory_data_.circle_pos_input[0]);
  if (!console_ui_->trajectory_data_.circle_pos_input[1].empty())
    console_ui_->trajectory_data_.circle[1] = std::stod(console_ui_->trajectory_data_.circle_pos_input[1]);
  if (!console_ui_->trajectory_data_.circle_pos_input[2].empty())
    console_ui_->trajectory_data_.circle[2] = std::stod(console_ui_->trajectory_data_.circle_pos_input[2]);
  if (!console_ui_->trajectory_data_.circle_pos_input[3].empty())
    console_ui_->trajectory_data_.circle[3] = std::stod(console_ui_->trajectory_data_.circle_pos_input[3]);
  if (!console_ui_->trajectory_data_.circle_speed_input.empty())
    console_ui_->trajectory_data_.circle[4] = std::stod(console_ui_->trajectory_data_.circle_speed_input);

  violet_msgs::msg::Trajectory follow_msg;
  follow_msg.path_type = 2; // Circle
  follow_msg.circle[0] = console_ui_->trajectory_data_.circle[0];
  follow_msg.circle[1] = console_ui_->trajectory_data_.circle[1];  
  follow_msg.circle[2] = console_ui_->trajectory_data_.circle[2];
  follow_msg.circle[3] = console_ui_->trajectory_data_.circle[3];
  follow_msg.circle[4] = console_ui_->trajectory_data_.circle[4];
  follow_pub_->publish(follow_msg);
}

void ConsoleNode::on_lemniscate_click() {
  if (!console_ui_->trajectory_data_.lemniscate_pos_input[0].empty())
    console_ui_->trajectory_data_.lemniscate[0] = std::stod(console_ui_->trajectory_data_.lemniscate_pos_input[0]);
  if (!console_ui_->trajectory_data_.lemniscate_pos_input[1].empty())
    console_ui_->trajectory_data_.lemniscate[1] = std::stod(console_ui_->trajectory_data_.lemniscate_pos_input[1]);
  if (!console_ui_->trajectory_data_.lemniscate_pos_input[2].empty())
    console_ui_->trajectory_data_.lemniscate[2] = std::stod(console_ui_->trajectory_data_.lemniscate_pos_input[2]);
  if (!console_ui_->trajectory_data_.lemniscate_pos_input[3].empty())
    console_ui_->trajectory_data_.lemniscate[3] = std::stod(console_ui_->trajectory_data_.lemniscate_pos_input[3]);
  if (!console_ui_->trajectory_data_.lemniscate_speed_input.empty())
    console_ui_->trajectory_data_.lemniscate[4] = std::stod(console_ui_->trajectory_data_.lemniscate_speed_input);

  violet_msgs::msg::Trajectory follow_msg;
  follow_msg.path_type = 3; // Lemniscata
  follow_msg.lemniscate[0] = console_ui_->trajectory_data_.lemniscate[0];
  follow_msg.lemniscate[1] = console_ui_->trajectory_data_.lemniscate[1];  
  follow_msg.lemniscate[2] = console_ui_->trajectory_data_.lemniscate[2];
  follow_msg.lemniscate[3] = console_ui_->trajectory_data_.lemniscate[3];
  follow_msg.lemniscate[4] = console_ui_->trajectory_data_.lemniscate[4];
  follow_pub_->publish(follow_msg);
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

