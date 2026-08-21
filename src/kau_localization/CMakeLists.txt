cmake_minimum_required(VERSION 3.8)
project(kau_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# SLAM/localization 본체는 cartographer_ros 가 제공한다.
# 이 패키지가 직접 빌드하는 것은 초기 위치 릴레이 노드 하나뿐.
add_executable(initial_pose_relay src/initial_pose_relay.cpp)
ament_target_dependencies(initial_pose_relay
  rclcpp
  cartographer_ros_msgs
  geometry_msgs
)

install(TARGETS
  initial_pose_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
