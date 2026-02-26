#include "rclcpp/rclcpp.hpp"
#include "ros_node.hpp"

/**
 * @brief The entrypoint of the mavlink driver node
 * 
 * @param argc The number of arguments passed to the program
 * @param argv The argument string array passed to the program
 * @return int The execution error returned to the user
 */
int main(int argc, char ** argv) {

    // Initiate ROS2 library
    rclcpp::init(argc, argv);

    try {
        // Initialize the ros2 communication side
        auto ros_node = std::make_shared<ROSNode>();
        ros_node->start();
        rclcpp::spin(ros_node);
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("violet_interface"), "Fatal startup/runtime error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
