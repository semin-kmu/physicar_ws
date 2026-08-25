#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "backup_lane_detection/lane_detector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<backup_lane_detection::LaneDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
