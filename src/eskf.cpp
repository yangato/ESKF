// src/eskf.cpp
#include <eskf/Node.hpp>
#include <rclcpp/rclcpp.hpp>
#include<cstdint>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<eskf::eskf_node>();  // <-- use your class
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}