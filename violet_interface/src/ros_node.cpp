#include <Eigen/Dense>
#include <cmath>
#include "ros_node.hpp"

ROSNode::ROSNode() : rclcpp::Node("interface_node") {}

ROSNode::~ROSNode() {}

void ROSNode::start() {

  // initialise parameters, subs, pubs and services
  init_parameters();
  init_publishers();
  init_subscribers_and_services();

}

void ROSNode::init_parameters() {
  this->declare_parameter<int>("vehicle_id", 1);
  vehicle_id_ = this->get_parameter("vehicle_id").as_int();
}

void ROSNode::init_publishers() {
  /**
   * Publisher for battery status
   */
  this->declare_parameter<std::string>("publishers.telemetry.battery", "fmu/telemetry/battery");
  rclcpp::Parameter battery_topic = this->get_parameter("publishers.telemetry.battery");
  battery_pub_ = this->create_publisher<violet_msgs::msg::Battery>
    (battery_topic.as_string(), rclcpp::SensorDataQoS());
  /**
   * Publisher for state
   */
  this->declare_parameter<std::string>("publishers.telemetry.state", "fmu/telemetry/state");
  rclcpp::Parameter state_topic = this->get_parameter("publishers.telemetry.state");
  state_pub_ = this->create_publisher<violet_msgs::msg::State>
    (state_topic.as_string(), rclcpp::SensorDataQoS());
  /**
   * Publisher for vehicle status
   */
  this->declare_parameter<std::string>("publishers.telemetry.status", "fmu/telemetry/status");
  rclcpp::Parameter status_topic = this->get_parameter("publishers.telemetry.status");
  status_pub_ = this->create_publisher<violet_msgs::msg::Status>
    (status_topic.as_string(), rclcpp::SensorDataQoS());
  /**
   * Publisher for vehicle mode
   */
  this->declare_parameter<std::string>("publishers.mode.request", "fmu/in/vehicle_command");
  rclcpp::Parameter mode_topic = this->get_parameter("publishers.mode.request");
  mode_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>
    (mode_topic.as_string(), rclcpp::SensorDataQoS());
    
}

void ROSNode::init_subscribers_and_services() {
  srv_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  ack_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  /**
   * Subscribe to battery status
   */
  this->declare_parameter<std::string>("subscribers.telemetry.battery", "fmu/out/battery_status_v1");
  rclcpp::Parameter battery_topic = this->get_parameter("subscribers.telemetry.battery");
  battery_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
    battery_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ROSNode::on_battery_callback, this, std::placeholders::_1));
  /**
   * Subscribe to state
   */
  this->declare_parameter<std::string>("subscribers.telemetry.state", "fmu/out/vehicle_odometry");
  rclcpp::Parameter state_topic = this->get_parameter("subscribers.telemetry.state");
  state_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    state_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ROSNode::on_state_callback, this, std::placeholders::_1));
  /**
   * Subscribe to vehicle status
   */
  this->declare_parameter<std::string>("subscribers.telemetry.status", "fmu/out/vehicle_status_v2");
  rclcpp::Parameter status_topic = this->get_parameter("subscribers.telemetry.status");
  status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    status_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ROSNode::on_status_callback, this, std::placeholders::_1));
  /**
   * Subscribe to vehicle mode ack
   */
  this->declare_parameter<std::string>("subscribers.mode.response", "fmu/out/vehicle_command_ack_v1");
  rclcpp::Parameter mode_topic = this->get_parameter("subscribers.mode.response");
  rclcpp::SubscriptionOptions mode_sub_options;
  mode_sub_options.callback_group = ack_group_;
  mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
    mode_topic.as_string(),
    rclcpp::SensorDataQoS(),
    std::bind(&ROSNode::mode_ack_callback, this, std::placeholders::_1),
    mode_sub_options);
  
  /**
   * Arm service
   */
  this->declare_parameter<std::string>("services.mode.arm", "fmu/mode/arm");
  rclcpp::Parameter arm_topic = this->get_parameter("services.mode.arm");
  arm_srv_ = this->create_service<violet_msgs::srv::Mode>(
    arm_topic.as_string(),
    std::bind(&ROSNode::srv_arm_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    srv_group_);
  /**
   * Disarm service
   */
  this->declare_parameter<std::string>("services.mode.disarm", "fmu/mode/disarm");
  rclcpp::Parameter disarm_topic = this->get_parameter("services.mode.disarm");
  disarm_srv_ = this->create_service<violet_msgs::srv::Mode>(
    disarm_topic.as_string(),
    std::bind(&ROSNode::srv_disarm_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    srv_group_);
  /**
   * Takeoff service
   */
  this->declare_parameter<std::string>("services.mode.takeoff", "fmu/mode/takeoff");
  rclcpp::Parameter takeoff_topic = this->get_parameter("services.mode.takeoff");
  takeoff_srv_ = this->create_service<violet_msgs::srv::Mode>(
    takeoff_topic.as_string(),
    std::bind(&ROSNode::srv_takeoff_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    srv_group_);
  /**
   * Loiter service
   */
  this->declare_parameter<std::string>("services.mode.loiter", "fmu/mode/loiter");
  rclcpp::Parameter loiter_topic = this->get_parameter("services.mode.loiter");
  loiter_srv_ = this->create_service<violet_msgs::srv::Mode>(
    loiter_topic.as_string(),
    std::bind(&ROSNode::srv_loiter_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    srv_group_);
  /**
   * Land service
   */
  this->declare_parameter<std::string>("services.mode.land", "fmu/mode/land");
  rclcpp::Parameter land_topic = this->get_parameter("services.mode.land");
  land_srv_ = this->create_service<violet_msgs::srv::Mode>(
    land_topic.as_string(),
    std::bind(&ROSNode::srv_land_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    srv_group_);
  /**
   * Kill service
   */
  this->declare_parameter<std::string>("services.mode.kill", "fmu/mode/kill");
  rclcpp::Parameter kill_topic = this->get_parameter("services.mode.kill");
  kill_srv_ = this->create_service<violet_msgs::srv::Mode>(
    kill_topic.as_string(),
    std::bind(&ROSNode::srv_kill_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    srv_group_);
}

void ROSNode::on_battery_callback(const px4_msgs::msg::BatteryStatus::ConstSharedPtr msg) { 

  // Set the current timestamp
  battery_msg_.header.stamp = rclcpp::Clock().now();

  // Set battery fields
  battery_msg_.soc = msg->remaining;
  battery_msg_.voltage = msg->voltage_v;
  battery_msg_.current = msg->current_a;

  // Publish message
  battery_pub_->publish(battery_msg_);
}

void ROSNode::on_state_callback(const px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
  
  const auto &pos     = msg->position;          
  const auto &q       = msg->q;                 
  const auto &vel     = msg->velocity;          
  const auto &ang_vel = msg->angular_velocity; 

  // Set the current timestamp
  state_msg_.header.stamp = rclcpp::Clock().now();

  // position 
  state_msg_.position[0] = pos[0]; // N
  state_msg_.position[1] = pos[1]; // E
  state_msg_.position[2] = pos[2]; // D

  // velocity
  state_msg_.inertial_velocity[0] = vel[0]; // N
  state_msg_.inertial_velocity[1] = vel[1]; // E
  state_msg_.inertial_velocity[2] = vel[2]; // D

  // attitude
  Eigen::Vector3f rpy;

  /* Compute roll */
  rpy[0] = std::atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2]*q[2]));
  float sin_pitch = 2 * (q[0]*q[2] - q[3]*q[1]);
  sin_pitch = sin_pitch >  1 ?  1 : sin_pitch;
  sin_pitch = sin_pitch < -1 ? -1 : sin_pitch;

  /* Compute pitch */
  rpy[1]= std::asin(sin_pitch);

  /* Compute yaw */
  rpy[2] = std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));

  state_msg_.attitude[0] = rpy[0];
  state_msg_.attitude[1] = rpy[1];
  state_msg_.attitude[2] = rpy[2];

  // angular velocity
  state_msg_.angular_velocity[0] = ang_vel[0];
  state_msg_.angular_velocity[1] = ang_vel[1];
  state_msg_.angular_velocity[2] = ang_vel[2];

  // Publish message
  state_pub_->publish(state_msg_);
}

void ROSNode::on_status_callback(const px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) { 
  
  // Set the current timestamp
  status_msg_.header.stamp = rclcpp::Clock().now();

  // Set vehicle id
  status_msg_.id = vehicle_id_;

  // Set armed state
  switch (msg->arming_state) {
  case 1:
    status_msg_.armed = false;
    break;
  case 2:
    status_msg_.armed = true;
    break;
  }

  // Set mode 
  switch (msg->nav_state) {
    case 17: //takeoff
      status_msg_.mode = 0;
      break;
    case 18: //land
      status_msg_.mode = 1;
      break;
    case 4: //loiter
      status_msg_.mode = 2;
      break;
    default:
      status_msg_.mode = 255; 
      break;
  }

  // Publish message
  status_pub_->publish(status_msg_);
}

void ROSNode::mode_ack_callback(const px4_msgs::msg::VehicleCommandAck::ConstSharedPtr msg) {
  if (msg->result == 0){
    std::lock_guard<std::mutex> lock(ack_mutex_);
    got_ack_ = true;
    ack_cv_.notify_all();
  }
}

void ROSNode::srv_arm_callback(const violet_msgs::srv::Mode::Request::SharedPtr request, const violet_msgs::srv::Mode::Response::SharedPtr response) {
  (void)request;
  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    got_ack_ = false;
  }
  response->success = false;

  // Build Arm request
  px4_msgs::msg::VehicleCommand msg{};

  msg.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL);  // microseconds

  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM; // 400
  msg.param1 = 1.0f;  //arm

  msg.target_system = vehicle_id_;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;

  msg.from_external = true;

  mode_pub_->publish(msg);

  // Wait up to 3s for a matching ACK while other callbacks keep running.
  std::unique_lock<std::mutex> lock(ack_mutex_);
  const bool ack_received = ack_cv_.wait_for(
    lock,
    std::chrono::seconds(3),
    [this]() { return got_ack_; });

  response->success = ack_received;
  got_ack_ = false;
}

void ROSNode::srv_disarm_callback(const violet_msgs::srv::Mode::Request::SharedPtr request, const violet_msgs::srv::Mode::Response::SharedPtr response) {
  (void)request;
  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    got_ack_ = false;
  }
  response->success = false;

  // Build Disarm request
  px4_msgs::msg::VehicleCommand msg{};

  msg.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL);  // microseconds

  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM; // 400
  msg.param1 = 0.0f;  //disarm

  msg.target_system = vehicle_id_;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;

  msg.from_external = true;

  mode_pub_->publish(msg);

  // Wait up to 3s for a matching ACK while other callbacks keep running.
  std::unique_lock<std::mutex> lock(ack_mutex_);
  const bool ack_received = ack_cv_.wait_for(
    lock,
    std::chrono::seconds(3),
    [this]() { return got_ack_; });

  response->success = ack_received;
  got_ack_ = false;
}

void ROSNode::srv_takeoff_callback(const violet_msgs::srv::Mode::Request::SharedPtr request, const violet_msgs::srv::Mode::Response::SharedPtr response) {
  (void)request;
  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    got_ack_ = false;
  }

  response->success = false;

  px4_msgs::msg::VehicleCommand msg{};

  msg.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL); // microseconds

  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE; // 176
  msg.param1 = 1.0f;  // use custom mode
  msg.param2 = 4.0f;  // PX4_CUSTOM_MAIN_MODE_AUTO
  msg.param3 = 2.0f;  // takeoff

  msg.target_system = vehicle_id_;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;

  msg.from_external = true;

  mode_pub_->publish(msg);

  std::unique_lock<std::mutex> lock(ack_mutex_);
  const bool ack_received = ack_cv_.wait_for(
    lock,
    std::chrono::seconds(3),
    [this]() { return got_ack_; });

  response->success = ack_received;
  got_ack_ = false;
}

void ROSNode::srv_loiter_callback(const violet_msgs::srv::Mode::Request::SharedPtr request, const violet_msgs::srv::Mode::Response::SharedPtr response) {
  (void)request;
  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    got_ack_ = false;
  }

  response->success = false;

  px4_msgs::msg::VehicleCommand msg{};

  msg.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL); // microseconds

  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE; 
  msg.param1 = 1.0f;  // use custom mode
  msg.param2 = 4.0f;  // PX4_CUSTOM_MAIN_MODE_AUTO
  msg.param3 = 3.0f;  // loiter


  msg.target_system = vehicle_id_;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;

  msg.from_external = true;

  mode_pub_->publish(msg);

  std::unique_lock<std::mutex> lock(ack_mutex_);
  const bool ack_received = ack_cv_.wait_for(
    lock,
    std::chrono::seconds(3),
    [this]() { return got_ack_; });

  response->success = ack_received;
  got_ack_ = false;
}

void ROSNode::srv_land_callback(const violet_msgs::srv::Mode::Request::SharedPtr request, const violet_msgs::srv::Mode::Response::SharedPtr response) {
  (void)request;
  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    got_ack_ = false;
  }

  response->success = false;

  px4_msgs::msg::VehicleCommand msg{};

  msg.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL); // microseconds

  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE; 
  msg.param1 = 1.0f;  // use custom mode
  msg.param2 = 4.0f;  // PX4_CUSTOM_MAIN_MODE_AUTO
  msg.param3 = 6.0f;  // land


  msg.target_system = vehicle_id_;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;

  msg.from_external = true;

  mode_pub_->publish(msg);

  std::unique_lock<std::mutex> lock(ack_mutex_);
  const bool ack_received = ack_cv_.wait_for(
    lock,
    std::chrono::seconds(3),
    [this]() { return got_ack_; });

  response->success = ack_received;
  got_ack_ = false;
}

void ROSNode::srv_kill_callback(
  const violet_msgs::srv::Mode::Request::SharedPtr request,
  const violet_msgs::srv::Mode::Response::SharedPtr response)
{
  (void)request;
  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    got_ack_ = false;
  }

  response->success = false;

  px4_msgs::msg::VehicleCommand msg{};

  msg.timestamp =
    static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL);

  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM; // 400
  msg.param1  = 0.0f;     // disarm
  msg.param2  = 21196.0f; // force disarm (kill)

  msg.target_system    = vehicle_id_;
  msg.target_component = 1;
  msg.source_system    = 1;
  msg.source_component = 1;

  msg.from_external = true;

  mode_pub_->publish(msg);

  std::unique_lock<std::mutex> lock(ack_mutex_);
  const bool ack_received = ack_cv_.wait_for(
    lock,
    std::chrono::seconds(3),
    [this]() { return got_ack_; });

  response->success = ack_received;
  got_ack_ = false;
}
