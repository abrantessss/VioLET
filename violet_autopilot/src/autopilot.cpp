#include "autopilot.hpp"

Autopilot::Autopilot() : rclcpp::Node("autopilot_node") {}

Autopilot::~Autopilot() {}

void Autopilot::start() {

  // initialise parameters, subs, pubs and services
  init_publishers();
  init_subscribers_and_services();
}

void Autopilot::init_publishers() {
  /**
   * Publisher for arm mode
   */
  this->declare_parameter<std::string>("publishers.mode.arm", "fmu/mode/arm");
  rclcpp::Parameter arm_topic = this->get_parameter("publishers.mode.arm");
  arm_client_ = this->create_client<violet_msgs::srv::Mode>
    (arm_topic.as_string());
  /**
   * Publisher for disarm mode
   */
  this->declare_parameter<std::string>("publishers.mode.disarm", "fmu/mode/disarm");
  rclcpp::Parameter disarm_topic = this->get_parameter("publishers.mode.disarm");
  disarm_client_ = this->create_client<violet_msgs::srv::Mode>
    (disarm_topic.as_string());
  /**
   * Publisher for takeoff mode
   */
  this->declare_parameter<std::string>("publishers.mode.takeoff", "fmu/mode/takeoff");
  rclcpp::Parameter takeoff_topic = this->get_parameter("publishers.mode.takeoff");
  takeoff_client_ = this->create_client<violet_msgs::srv::Mode>
    (takeoff_topic.as_string());
  /**
   * Publisher for loiter mode
   */
  this->declare_parameter<std::string>("publishers.mode.loiter", "fmu/mode/loiter");
  rclcpp::Parameter loiter_topic = this->get_parameter("publishers.mode.loiter");
  loiter_client_ = this->create_client<violet_msgs::srv::Mode>
    (loiter_topic.as_string());
  /**
   * Publisher for land mode
   */
  this->declare_parameter<std::string>("publishers.mode.land", "fmu/mode/land");
  rclcpp::Parameter land_topic = this->get_parameter("publishers.mode.land");
  land_client_ = this->create_client<violet_msgs::srv::Mode>
    (land_topic.as_string());
  /**
   * Publisher for kill mode
   */
  this->declare_parameter<std::string>("publishers.mode.kill", "fmu/mode/kill");
  rclcpp::Parameter kill_topic = this->get_parameter("publishers.mode.kill");
  kill_client_ = this->create_client<violet_msgs::srv::Mode>
    (kill_topic.as_string());
}


void Autopilot::init_subscribers_and_services() {
  /**
   * Subscribe to arm request message
   */ 
  this->declare_parameter<std::string>("subscribers.mode.arm", "fmu/mode/arm");
  rclcpp::Parameter arm_topic = this->get_parameter("subscribers.mode.arm");
  arm_sub_ = this->create_subscription<violet_msgs::msg::Mode>(
    arm_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::on_arm_callback, this, std::placeholders::_1));

  /**
   * Subscribe to disarm request message
   */ 
  this->declare_parameter<std::string>("subscribers.mode.disarm", "fmu/mode/disarm");
  rclcpp::Parameter disarm_topic = this->get_parameter("subscribers.mode.disarm");
  disarm_sub_ = this->create_subscription<violet_msgs::msg::Mode>(
    disarm_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::on_disarm_callback, this, std::placeholders::_1));

  /**
   * Subscribe to takeoff request message
   */ 
  this->declare_parameter<std::string>("subscribers.mode.takeoff", "fmu/mode/takeoff");
  rclcpp::Parameter takeoff_topic = this->get_parameter("subscribers.mode.takeoff");
  takeoff_sub_ = this->create_subscription<violet_msgs::msg::Mode>(
    takeoff_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::on_takeoff_callback, this, std::placeholders::_1));

  /**
   * Subscribe to loiter request message
   */ 
  this->declare_parameter<std::string>("subscribers.mode.loiter", "fmu/mode/loiter");
  rclcpp::Parameter loiter_topic = this->get_parameter("subscribers.mode.loiter");
  loiter_sub_ = this->create_subscription<violet_msgs::msg::Mode>(
    loiter_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::on_loiter_callback, this, std::placeholders::_1));

  /**
   * Subscribe to land request message
   */ 
  this->declare_parameter<std::string>("subscribers.mode.land", "fmu/mode/land");
  rclcpp::Parameter land_topic = this->get_parameter("subscribers.mode.land");
  land_sub_ = this->create_subscription<violet_msgs::msg::Mode>(
    land_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::on_land_callback, this, std::placeholders::_1));

  /**
   * Subscribe to kill request message
   */ 
  this->declare_parameter<std::string>("subscribers.mode.kill", "fmu/mode/kill");
  rclcpp::Parameter kill_topic = this->get_parameter("subscribers.mode.kill");
  kill_sub_ = this->create_subscription<violet_msgs::msg::Mode>(
    kill_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::on_kill_callback, this, std::placeholders::_1));
}

void Autopilot::on_arm_callback(const violet_msgs::msg::Mode::ConstSharedPtr msg) {
  (void)msg;

  // Arm the vehicle by invoking the service 
  auto arm_request = std::make_shared<violet_msgs::srv::Mode::Request>();

  // Wait until the service is available
  while (!arm_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  // Send the request asynchronously and process the response in-place.
  arm_client_->async_send_request(
    arm_request,
    [this](rclcpp::Client<violet_msgs::srv::Mode>::SharedFuture future) {
      const auto response = future.get();
      if (response->success)
        RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully");
      else
        RCLCPP_WARN(this->get_logger(), "Vehicle failed to arm");
    }
  );

  current_mode_ = 0;
}

void Autopilot::on_disarm_callback(const violet_msgs::msg::Mode::ConstSharedPtr msg) {
  (void)msg;
  // Disarm the vehicle by invoking the service 
  auto disarm_request = std::make_shared<violet_msgs::srv::Mode::Request>();

  // Wait until the service is available
  while (!disarm_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  // Send the request asynchronously and process the response in-place.
  disarm_client_->async_send_request(
    disarm_request,
    [this](rclcpp::Client<violet_msgs::srv::Mode>::SharedFuture future) {
      const auto response = future.get();
      if (response->success)
        RCLCPP_INFO(this->get_logger(), "Vehicle disarmed successfully");
      else
        RCLCPP_WARN(this->get_logger(), "Vehicle failed to disarm");
    }
  );

  current_mode_ = 1;
}

void Autopilot::on_takeoff_callback(const violet_msgs::msg::Mode::ConstSharedPtr msg) {
  (void)msg;
  // Takeoff by invoking the service 
  auto takeoff_request = std::make_shared<violet_msgs::srv::Mode::Request>();

  // Wait until the service is available
  while (!takeoff_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  // Send the request asynchronously and process the response in-place.
  takeoff_client_->async_send_request(
    takeoff_request,
    [this](rclcpp::Client<violet_msgs::srv::Mode>::SharedFuture future) {
      const auto response = future.get();
      if (response->success)
        RCLCPP_INFO(this->get_logger(), "Vehicle takeoff successfully");
      else
        RCLCPP_WARN(this->get_logger(), "Vehicle failed to takeoff");
    }
  );

  current_mode_ = 2;
}

void Autopilot::on_loiter_callback(const violet_msgs::msg::Mode::ConstSharedPtr msg) {
  (void)msg;
  // Loiter by invoking the service 
  auto loiter_request = std::make_shared<violet_msgs::srv::Mode::Request>();

  // Wait until the service is available
  while (!loiter_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  // Send the request asynchronously and process the response in-place.
  loiter_client_->async_send_request(
    loiter_request,
    [this](rclcpp::Client<violet_msgs::srv::Mode>::SharedFuture future) {
      const auto response = future.get();
      if (response->success)
        RCLCPP_INFO(this->get_logger(), "Vehicle Loiter successfully");
      else
        RCLCPP_WARN(this->get_logger(), "Vehicle failed to loiter");
    }
  );

  current_mode_ = 3;
}

void Autopilot::on_land_callback(const violet_msgs::msg::Mode::ConstSharedPtr msg) {
  (void)msg;
  // Land by invoking the service 
  auto land_request = std::make_shared<violet_msgs::srv::Mode::Request>();

  // Wait until the service is available
  while (!land_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  // Send the request asynchronously and process the response in-place.
  land_client_->async_send_request(
    land_request,
    [this](rclcpp::Client<violet_msgs::srv::Mode>::SharedFuture future) {
      const auto response = future.get();
      if (response->success)
        RCLCPP_INFO(this->get_logger(), "Vehicle Land successfully");
      else
        RCLCPP_WARN(this->get_logger(), "Vehicle failed to land");
    }
  );

  current_mode_ = 4;
}

void Autopilot::on_kill_callback(const violet_msgs::msg::Mode::ConstSharedPtr msg) {
  (void)msg;
  // Kill by invoking the service 
  auto kill_request = std::make_shared<violet_msgs::srv::Mode::Request>();

  // Wait until the service is available
  while (!kill_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  // Send the request asynchronously and process the response in-place.
  kill_client_->async_send_request(
    kill_request,
    [this](rclcpp::Client<violet_msgs::srv::Mode>::SharedFuture future) {
      const auto response = future.get();
      if (response->success)
        RCLCPP_INFO(this->get_logger(), "Vehicle Kill successfully");
      else
        RCLCPP_WARN(this->get_logger(), "Vehicle failed to kill");
    }
  );

  current_mode_ = 5;
}
