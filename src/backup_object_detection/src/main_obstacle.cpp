#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "backup_object_detection/obstacle_detector_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<backup_object_detection::ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
