#pragma once

#include <condition_variable>
#include <mutex>

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

// Modes
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "violet_msgs/srv/mode.hpp"

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
     * @ingroup publisherMessageUpdate
     * @brief Method used to update mode change ack
     */
    void mode_ack_callback(const px4_msgs::msg::VehicleCommandAck::ConstSharedPtr msg);



    /**
     * @defgroup servicesCallbacks
     * This group defines all the callbacks of the services
     */
    
    /**
     * @ingroup servicesCallbacks
     * @brief Arm service callback 
     */ 
    void srv_arm_callback(const violet_msgs::srv::Mode::Request::SharedPtr request, const violet_msgs::srv::Mode::Response::SharedPtr response);

    /**
     * @ingroup servicesCallbacks
     * @brief Disarm service callback 
     */ 
    void srv_disarm_callback(const violet_msgs::srv::Mode::Request::SharedPtr request, const violet_msgs::srv::Mode::Response::SharedPtr response);

    /**
     * @ingroup servicesCallbacks
     * @brief Takeoff service callback 
     */ 
    void srv_takeoff_callback(const violet_msgs::srv::Mode::Request::SharedPtr request, const violet_msgs::srv::Mode::Response::SharedPtr response);

    /**
     * @ingroup servicesCallbacks
     * @brief Loiter service callback 
     */ 
    void srv_loiter_callback(const violet_msgs::srv::Mode::Request::SharedPtr request, const violet_msgs::srv::Mode::Response::SharedPtr response);

    /**
     * @ingroup servicesCallbacks
     * @brief Land service callback 
     */ 
    void srv_land_callback(const violet_msgs::srv::Mode::Request::SharedPtr request, const violet_msgs::srv::Mode::Response::SharedPtr response);

    /**
     * @ingroup servicesCallbacks
     * @brief Kill service callback 
     */ 
    void srv_kill_callback(const violet_msgs::srv::Mode::Request::SharedPtr request, const violet_msgs::srv::Mode::Response::SharedPtr response);



    /**
     * @defgroup messages
     * This group defines all the ROS messages
     */
    violet_msgs::msg::Battery battery_msg_;
    violet_msgs::msg::State state_msg_;
    violet_msgs::msg::Status status_msg_;



    /**
     * @defgroup publisher 
     * This group defines all the ROS publisher
     */
    rclcpp::Publisher<violet_msgs::msg::Battery>::SharedPtr battery_pub_{nullptr};
    rclcpp::Publisher<violet_msgs::msg::State>::SharedPtr state_pub_{nullptr};
    rclcpp::Publisher<violet_msgs::msg::Status>::SharedPtr status_pub_{nullptr};

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr mode_pub_{nullptr};


    /**
     * @ingroup subscribers
     * Subscriber for drone status
     */
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_{nullptr};
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr state_sub_{nullptr};
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_{nullptr};
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr mode_sub_{nullptr};



    /**
     * @ingroup services
     * Services for used for modes
     */
    rclcpp::Service<violet_msgs::srv::Mode>::SharedPtr arm_srv_{nullptr};
    rclcpp::Service<violet_msgs::srv::Mode>::SharedPtr disarm_srv_{nullptr};
    rclcpp::Service<violet_msgs::srv::Mode>::SharedPtr takeoff_srv_{nullptr};
    rclcpp::Service<violet_msgs::srv::Mode>::SharedPtr loiter_srv_{nullptr};
    rclcpp::Service<violet_msgs::srv::Mode>::SharedPtr land_srv_{nullptr};
    rclcpp::Service<violet_msgs::srv::Mode>::SharedPtr kill_srv_{nullptr};

    rclcpp::CallbackGroup::SharedPtr ack_group_{nullptr};
    rclcpp::CallbackGroup::SharedPtr srv_group_{nullptr};


    /**
     * @ingroup Variables
     * variables used in the node
     */
    int vehicle_id_ {0};
    bool got_ack_ {false};
    std::mutex ack_mutex_;
    std::condition_variable ack_cv_;

};
