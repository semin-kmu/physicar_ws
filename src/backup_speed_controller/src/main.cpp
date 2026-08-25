#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "backup_speed_controller/speed_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<backup_speed_controller::SpeedControllerNode>();
  rclcpp::spin(node);
  // 종료 0 발행은 pre-shutdown 콜백에서 끝난다 (컨텍스트가 살아 있어야 나간다).
  rclcpp::shutdown();
  return 0;
}
