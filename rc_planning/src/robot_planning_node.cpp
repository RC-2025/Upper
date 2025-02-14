#include <rc_planning/robot_planning_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<robot::planning_node>();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}