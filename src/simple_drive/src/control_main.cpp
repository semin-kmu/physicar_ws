#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "simple_drive/control_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_drive::ControlNode>());
  rclcpp::shutdown();
  return 0;
}
