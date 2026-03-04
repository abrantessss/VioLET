#pragma once

#include "rclcpp/rclcpp.hpp"

// Console mode messages
#include "violet_msgs/msg/mode.hpp"


// Services mode
#include "violet_msgs/srv/mode.hpp"

// Maps
const std::map<uint8_t, std::string> mode_map = {
  {0  , "ARM"},
  {1  , "DISARM"},
  {2  , "TAKEOFF"},
  {3  , "LOITER"},
  {4  , "LAND"},
  {5  , "KILL"}
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



    /**
     * @ingroup Variables&Classes
     * variables and classed used in the node
     */
    int current_mode_{1};
};