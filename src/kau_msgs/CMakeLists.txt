cmake_minimum_required(VERSION 3.8)
project(kau_msgs)

# CMake 의 기본 빌드 타입은 "빈 문자열" 이고, 그러면 GCC 에 -O 가
# 아예 안 붙는다 (= -O0).
# 메시지 직렬화 코드가 -O0 으로 빌드되면 이 메시지를 쓰는 모든 노드가
# 같이 느려진다.
# 2026-08-25: build/ 를 훑어보니 이 패키지가 실제로 그 상태였다
# (kau_control/kau_lane_detection 만 Release, 나머지는 빈 값).
# --cmake-args 없이 `colcon build` 해도 안전하도록 기본값을 박아 둔다.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KauPath.msg"
  "msg/ObstacleCircle.msg"
  "msg/ObstacleCircleArray.msg"
  "msg/SteerDebug.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
