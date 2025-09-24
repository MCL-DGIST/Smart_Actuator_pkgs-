#include "ros_interface.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ElmoRosInterface>();

  RCLCPP_INFO(node->get_logger(), "Starting ELMO ROS Interface...");

  try {
    // rclcpp::spin(node);
    node->update();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Node crashed: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
