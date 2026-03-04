#include "rclcpp/rclcpp.hpp"
#include "autopilot.hpp"

/**
 * @brief The entrypoint of the autopilot node
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
      auto autopilot_node = std::make_shared<Autopilot>();
      autopilot_node->start();
      rclcpp::spin(autopilot_node);
  } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger("violet_autopilot"), "Fatal startup/runtime error: %s", e.what());
      rclcpp::shutdown();
      return 1;
  }

  rclcpp::shutdown();
  
  return 0;
}
