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
    
}

void ROSNode::init_subscribers_and_services() {
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
    case 21: //orbit
      status_msg_.mode = 1;
      break;
    case 18: //land
      status_msg_.mode = 2;
      break;
    case 4: //loiter
      status_msg_.mode = 3;
      break;
    default:
      status_msg_.mode = 255; 
      break;
  }

  // Publish message
  status_pub_->publish(status_msg_);
}