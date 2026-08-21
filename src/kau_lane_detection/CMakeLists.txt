cmake_minimum_required(VERSION 3.8)
project(kau_lane_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(kau_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(kau_lane_detection_node
  src/kau_lane_detection_node.cpp    # 진입점 (main)
  src/kau_lane_detection.cpp         # 멤버 함수 구현
)

ament_target_dependencies(kau_lane_detection_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  kau_msgs
  nav_msgs
  geometry_msgs
)

target_include_directories(kau_lane_detection_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(kau_lane_detection_node
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

install(TARGETS
  kau_lane_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# 웹 뷰어(파이썬)를 launch 에서 executable 로 부를 수 있게 설치
install(PROGRAMS
  scripts/lane_viewer.py
  scripts/hls_tuner.py
  scripts/lane_failure_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  web
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()