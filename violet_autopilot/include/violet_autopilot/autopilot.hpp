#pragma once

#include "rclcpp/rclcpp.hpp"
#include "controller.hpp"
#include "onboard_controller.hpp"
#include "mellinger_controller.hpp"
#include <Eigen/Dense>

// Console mode messages
#include "violet_msgs/msg/mode.hpp"
#include "violet_msgs/msg/state.hpp"
#include "violet_msgs/msg/trajectory.hpp"


// Services mode
#include "violet_msgs/srv/mode.hpp"

// Structures
struct State {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d attitude{Eigen::Vector3d::Zero()};
  Eigen::Vector3d inertial_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

struct Trajectories {
  int type;
  
  Eigen::Vector3d waypoint{Eigen::Vector3d::Zero()};

  Eigen::Matrix<double, 7, 1> line{Eigen::Matrix<double, 7, 1>::Zero()};

  Eigen::Matrix<double, 5, 1> circle{Eigen::Matrix<double, 5, 1>::Zero()};

  Eigen::Matrix<double, 5, 1> lemniscate{Eigen::Matrix<double, 5, 1>::Zero()};
};


enum class Mode : uint8_t {
  ARM = 0,
  DISARM = 1,
  TAKEOFF = 2,
  LOITER = 3,
  LAND = 4,
  KILL = 5,
  FOLLOW = 6,
  UNKNOWN = 255
};


class Autopilot : public rclcpp::Node {
  public:
    Autopilot();
    ~Autopilot();

  /**
   * @brief Method used to start the Autopilot Node
   */
  void start();

  /**
   * @brief Method used to update autopilot state depending on the current mode
   */
  void update();

  private:
    /**
     * @defgroup initFunctions
     * This group defines all the private initialisation functions
     * used to initialise ROS pub, subs and services
     */

    /**
     * @ingroup initFunction
     * @brief Method used to initialise ROS2 subscribers and services
     */
    void init_subscribers_and_services();

    /**
     * @ingroup initFunction
     * @brief Method used to initialise ROS2 publishers
     */
    void init_publishers();

    /**
     * @ingroup initFunction
     * @brief Method used to initialise controller
     */
    void init_controller();

    /**
     * @defgroup publisherMessageUpdate
     * This group defines and the methods used to update the current mode
     */

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method used to arm the vehicle on a callback
     */
    void on_arm_callback(const violet_msgs::msg::Mode::ConstSharedPtr msg); 

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method used to disarm the vehicle on a callback
     */
    void on_disarm_callback(const violet_msgs::msg::Mode::ConstSharedPtr msg); 

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method used to takeoff the vehicle on a callback
     */
    void on_takeoff_callback(const violet_msgs::msg::Mode::ConstSharedPtr msg); 

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method used to loiter the vehicle on a callback
     */
    void on_loiter_callback(const violet_msgs::msg::Mode::ConstSharedPtr msg); 

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method used to land the vehicle on a callback
     */
    void on_land_callback(const violet_msgs::msg::Mode::ConstSharedPtr msg); 

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method used to kill the vehicle on a callback
     */
    void on_kill_callback(const violet_msgs::msg::Mode::ConstSharedPtr msg); 

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method used to get the trajectory to follow
     */
    void on_follow_callback(const violet_msgs::msg::Trajectory::ConstSharedPtr msg); 

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method used to get state of the vehicle
     */
    void on_state_callback(const violet_msgs::msg::State::ConstSharedPtr msg); 

    

    /**
     * @defgroup publisehrs
     */
    rclcpp::Client<violet_msgs::srv::Mode>::SharedPtr arm_client_{nullptr};
    rclcpp::Client<violet_msgs::srv::Mode>::SharedPtr disarm_client_{nullptr};
    rclcpp::Client<violet_msgs::srv::Mode>::SharedPtr takeoff_client_{nullptr};
    rclcpp::Client<violet_msgs::srv::Mode>::SharedPtr loiter_client_{nullptr};
    rclcpp::Client<violet_msgs::srv::Mode>::SharedPtr land_client_{nullptr};
    rclcpp::Client<violet_msgs::srv::Mode>::SharedPtr kill_client_{nullptr};
    
    /**
     * @defgroup subscribers
     */
    rclcpp::Subscription<violet_msgs::msg::Mode>::SharedPtr arm_sub_{nullptr};
    rclcpp::Subscription<violet_msgs::msg::Mode>::SharedPtr disarm_sub_{nullptr};
    rclcpp::Subscription<violet_msgs::msg::Mode>::SharedPtr takeoff_sub_{nullptr};
    rclcpp::Subscription<violet_msgs::msg::Mode>::SharedPtr loiter_sub_{nullptr};
    rclcpp::Subscription<violet_msgs::msg::Mode>::SharedPtr land_sub_{nullptr};
    rclcpp::Subscription<violet_msgs::msg::Mode>::SharedPtr kill_sub_{nullptr};
    rclcpp::Subscription<violet_msgs::msg::Trajectory>::SharedPtr follow_sub_{nullptr};
    rclcpp::Subscription<violet_msgs::msg::State>::SharedPtr state_sub_{nullptr};

    
    /**
     * @ingroup Variables&Classes
     * variables and classed used in the node
     */
    Mode current_mode_{Mode::DISARM};
    State state_;
    Trajectories trajectory_;
    std::string controller_type_;
    autopilot::Controller::UniquePtr controller_{nullptr};
    double tnow_;
    double tprev_;
    

};
