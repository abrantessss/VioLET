#include <getopt.h>
#include <iostream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "console_node.hpp"

int main(int argc, char *argv[]) {
  
  // Get parameters
  int ch;
  unsigned int vehicle_id = 1;
  std::string vehicle_ns = "/drone";

  while((ch = getopt(argc, argv, "i:n:")) != -1) {
    switch (ch)
    {
    case 'i':
        try {
          vehicle_id = std::stoi(optarg);
        } catch (std::invalid_argument& e){
          std::cerr << "Invalid vehicle ID -> Must be a number" << std::endl;
          return 0;
        }
      break;
    case 'n':
        vehicle_ns = std::string("/") + std::string(optarg);
      break;
    default:
      return 0;
    }
  }
  
  //Init ROS2 node
  rclcpp::init(argc, argv);
  auto console_node = std::make_shared<ConsoleNode>(vehicle_ns, vehicle_id);
  console_node->start();
  rclcpp::shutdown();
  return 0;
}
