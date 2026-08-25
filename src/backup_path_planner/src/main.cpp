#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "backup_path_planner/path_planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<backup_path_planner::PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
