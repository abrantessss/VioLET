#pragma once

#include "rclcpp/rclcpp.hpp"

// Messages for telemetry
// Battery
#include "px4_msgs/msg/battery_status.hpp"
#include "violet_msgs/msg/battery.hpp"
// State
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "violet_msgs/msg/state.hpp"
// Status
#include "px4_msgs/msg/vehicle_status.hpp"
#include "violet_msgs/msg/status.hpp"

class ROSNode : public rclcpp::Node {
  public:
    ROSNode();
    ~ROSNode();

    /**
     * @brief Method used to start the ROS2 node
     */
    void start();   

  private:
    /**
     * @defgroup initFunctions
     * This group defines all the private initialization functions
     * used to initialize ROS pub, subs and services
     */

    /**
     * @ingroup initFunctions
     * @brief Method used to initialise ROS2 parameters from args
     */
    void init_parameters();

    /**
     * @ingroup initFunction
     * @brief Method used to initialise ROS2 publishers
     */
    void init_publishers();


    /**
     * @ingroup initFunction
     * @brief Method used to initialise ROS2 subscribers and services
     */
    void init_subscribers_and_services();



    /**
     * @defgroup publisherMessageUpdate
     * This group defines all the methods used to update the telemetry messages
     */

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method used to update battery status
     */
    void on_battery_callback(const px4_msgs::msg::BatteryStatus::ConstSharedPtr msg);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method used to update state status
     */
    void on_state_callback(const px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method used to update vehicle status
     */
    void on_status_callback(const px4_msgs::msg::VehicleStatus::ConstSharedPtr msg);



    /**
     * @defgroup messages
     * This group defines all the ROS messages
     */

    int vehicle_id_{0};

    violet_msgs::msg::Battery battery_msg_;
    violet_msgs::msg::State state_msg_;
    violet_msgs::msg::Status status_msg_;


    /**
     * @defgroup subscribers 
     * This group defines all the ROS subscribers
     */
    rclcpp::Publisher<violet_msgs::msg::Battery>::SharedPtr battery_pub_{nullptr};
    rclcpp::Publisher<violet_msgs::msg::State>::SharedPtr state_pub_{nullptr};
    rclcpp::Publisher<violet_msgs::msg::Status>::SharedPtr status_pub_{nullptr};

    /**
     * @ingroup subscribers
     * Subscriber for drone battery status
     */
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_{nullptr};
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr state_sub_{nullptr};
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_{nullptr};
};
