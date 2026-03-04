#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "ros_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto ros_node = std::make_shared<ROSNode>();
    ros_node->start();

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(ros_node);
    exec.spin();

  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("violet_interface"), "Fatal startup/runtime error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
