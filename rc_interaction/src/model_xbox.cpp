#include <rclcpp/rclcpp.hpp>

#include <rc_interaction/xbox360_node.hpp>
#include <rc_interaction/sbus_node.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<robot::xbox360_node>();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}