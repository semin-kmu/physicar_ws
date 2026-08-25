cmake_minimum_required(VERSION 3.8)
project(backup_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneLine.msg"
  "msg/LaneGeometry.msg"
  "msg/BackupPath.msg"
  "msg/ObstacleCircle.msg"
  "msg/ObstacleCircleArray.msg"
  "msg/ControlDebug.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
