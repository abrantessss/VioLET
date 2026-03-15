#include "autopilot.hpp"

Autopilot::Autopilot() : rclcpp::Node("autopilot_node") {}

Autopilot::~Autopilot() {}

void Autopilot::start() {

  // initialise parameters, subs, pubs and services
  init_publishers();
  init_subscribers_and_services();
  init_controller();
}

void Autopilot::update() {
  while (rclcpp::ok()) {
    if (current_mode_ == Mode::FOLLOW) {
      tnow_ = this->get_clock()->now().nanoseconds() / 1e9;
      double dt = tnow_ - tprev_;
      tprev_ = tnow_;
      
      if (controller_type_ == "onboard") {
        controller_->set_position(dt, state_.position);
      } else if (controller_type_ == "mellinger") {
        controller_->set_attitude_rate(dt, state_.position, state_.inertial_velocity, state_.attitude);
      } else {
        throw std::runtime_error("Unknown controller type: " + controller_type_);
      }
      
    } 
    rclcpp::spin_some(this->get_node_base_interface());
  }
}

void Autopilot::init_controller() {
  this->declare_parameter<std::string>("controllers.type", "onboard");
  controller_type_ = this->get_parameter("controllers.type").as_string();

  if (controller_type_ == "onboard") {
    controller_ = std::make_unique<autopilot::OnboardController>();
  } else if (controller_type_ == "mellinger") {
    controller_ = std::make_unique<autopilot::MellingerController>();
  } else {
    throw std::runtime_error("Unknown controller type: " + controller_type_);
  }

  autopilot::Controller::Config config;
  config.node = this->shared_from_this();

  controller_->initialize_controller(config);
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

  /**
   * Subscribe to follow message
   */ 
  this->declare_parameter<std::string>("subscribers.mode.follow", "fmu/mode/follow");
  rclcpp::Parameter follow_topic = this->get_parameter("subscribers.mode.follow");
  follow_sub_ = this->create_subscription<violet_msgs::msg::Trajectory>(
    follow_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::on_follow_callback, this, std::placeholders::_1));
  
  /**
   * Subscribe to state message
   */ 
  this->declare_parameter<std::string>("subscribers.state.state", "fmu/telemetry/state");
  rclcpp::Parameter state_topic = this->get_parameter("subscribers.state.state");
  state_sub_ = this->create_subscription<violet_msgs::msg::State>(
    state_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::on_state_callback, this, std::placeholders::_1));
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
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully");
        current_mode_ = Mode::ARM;
      }
      else
        RCLCPP_WARN(this->get_logger(), "Vehicle failed to arm");
    }
  );
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
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Vehicle disarmed successfully");
        current_mode_ = Mode::DISARM;
      }
      else
        RCLCPP_WARN(this->get_logger(), "Vehicle failed to disarm");
    }
  );
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
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Vehicle takeoff successfully");
        current_mode_ = Mode::TAKEOFF;
      }
      else
        RCLCPP_WARN(this->get_logger(), "Vehicle failed to takeoff");
    }
  );
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
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Vehicle Loiter successfully");
        current_mode_ = Mode::LOITER;
      }
      else
        RCLCPP_WARN(this->get_logger(), "Vehicle failed to loiter");
    }
  );
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
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Vehicle Land successfully");
        current_mode_ = Mode::LAND;
      }
      else
        RCLCPP_WARN(this->get_logger(), "Vehicle failed to land");
    }
  );
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
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Vehicle Kill successfully");
        current_mode_ = Mode::KILL;
      }
      else
        RCLCPP_WARN(this->get_logger(), "Vehicle failed to kill");
    }
  );
}

void Autopilot::on_follow_callback(const violet_msgs::msg::Trajectory::ConstSharedPtr msg) {
  trajectory_.type = msg->path_type;
  
  trajectory_.waypoint[0] = msg->waypoint[0];
  trajectory_.waypoint[1] = msg->waypoint[1];
  trajectory_.waypoint[2] = msg->waypoint[2];

  trajectory_.line[0] = msg->line[0];  
  trajectory_.line[1] = msg->line[1];
  trajectory_.line[2] = msg->line[2];
  trajectory_.line[3] = msg->line[3];
  trajectory_.line[4] = msg->line[4];
  trajectory_.line[5] = msg->line[5];
  trajectory_.line[6] = msg->line[6];

  trajectory_.circle[0] = msg->circle[0];
  trajectory_.circle[1] = msg->circle[1];
  trajectory_.circle[2] = msg->circle[2];
  trajectory_.circle[3] = msg->circle[3];
  trajectory_.circle[4] = msg->circle[4];

  trajectory_.lemniscate[0] = msg->lemniscate[0];
  trajectory_.lemniscate[1] = msg->lemniscate[1];
  trajectory_.lemniscate[2] = msg->lemniscate[2];
  trajectory_.lemniscate[3] = msg->lemniscate[3];
  trajectory_.lemniscate[4] = msg->lemniscate[4];

  current_mode_ = Mode::FOLLOW;

  if (trajectory_.type == 0) {
    controller_->set_path(trajectory_.type, trajectory_.waypoint.data());
  } else if (trajectory_.type == 1) {
    controller_->set_path(trajectory_.type, trajectory_.line.data());
  } else if (trajectory_.type == 2) {
    controller_->set_path(trajectory_.type, trajectory_.circle.data());
  } else {
    controller_->set_path(trajectory_.type, trajectory_.lemniscate.data());
  }

  tnow_ = this->get_clock()->now().nanoseconds() / 1e9;
  tprev_ = tnow_;
}

void Autopilot::on_state_callback(const violet_msgs::msg::State::ConstSharedPtr msg) {
  state_.position[0] = msg->position[0];
  state_.position[1] = msg->position[1];
  state_.position[2] = msg->position[2];

  state_.attitude[0] = msg->attitude[0];
  state_.attitude[1] = msg->attitude[1];
  state_.attitude[2] = msg->attitude[2];

  state_.inertial_velocity[0] = msg->inertial_velocity[0];
  state_.inertial_velocity[1] = msg->inertial_velocity[1];
  state_.inertial_velocity[2] = msg->inertial_velocity[2];

  state_.angular_velocity[0] = msg->angular_velocity[0];
  state_.angular_velocity[1] = msg->angular_velocity[1];
  state_.angular_velocity[2] = msg->angular_velocity[2];
}
