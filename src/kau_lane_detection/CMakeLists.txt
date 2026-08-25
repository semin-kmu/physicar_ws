cmake_minimum_required(VERSION 3.8)
project(kau_lane_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CMake 의 기본 빌드 타입은 "빈 문자열" 이고, 그러면 GCC 에 -O 가
# 아예 안 붙는다 (= -O0). 이 노드의 슬라이딩 윈도우는 프레임당
# 수십만 번 도는 순수 스칼라 루프라 -O0 이면 몇 배 느려진다.
# 지금 빌드 캐시에는 우연히 Release 가 들어 있지만, build/ 를 지우고
# 그냥 `colcon build` 하면 -O0 으로 되돌아간다. 기본값을 박아 둔다.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(kau_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
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
  tf2
  tf2_ros
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
  scripts/hls_snapshot_saver.py
  scripts/lane_failure_logger.py
  scripts/set_camera_info.py
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
  platform_overrides
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()