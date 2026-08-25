#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "backup_steer_controller/steer_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<backup_steer_controller::SteerControllerNode>());
  rclcpp::shutdown();
  return 0;
}
