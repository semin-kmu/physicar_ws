cmake_minimum_required(VERSION 3.8)
project(kau_control_drive)

# CMake 기본 빌드 타입은 빈 문자열이고, 그러면 GCC 에 -O 가 안 붙는다(-O0).
# 제어 루프가 매 tick Bezier 최근접점(9 차 근, Eigen 고유값)을 푼다.
# -O0 이면 그 한 번이 밀리초 단위로 뛴다 (kau_control_lane 과 같은 이유).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kau_msgs REQUIRED)
find_package(kau_control REQUIRED)
find_package(Eigen3 REQUIRED)

# 이 패키지의 헤더는 전부 header-only 다. 곡선 수학은 kau_control 것을
# 그대로 쓴다 (사본 없음).
add_library(kau_control_drive_core INTERFACE)
target_include_directories(kau_control_drive_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kau_control_drive_core INTERFACE Eigen3::Eigen)
ament_target_dependencies(kau_control_drive_core INTERFACE
  rclcpp kau_msgs kau_control)

foreach(node drive_steer_controller drive_speed_controller)
  add_executable(${node} src/${node}_node.cpp)
  target_link_libraries(${node} kau_control_drive_core)
  ament_target_dependencies(${node}
    rclcpp geometry_msgs nav_msgs std_msgs kau_msgs kau_control)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # 슬루 제한은 ROS 없이 검증한다. 기존 두 제어기가 max_steer_rate 를
  # 선언만 하고 안 쓰던 것을 실제로 거는 부분이라 회귀를 막아 둔다.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_steer_limiter test/test_steer_limiter.cpp)
  if(TARGET test_steer_limiter)
    target_include_directories(test_steer_limiter PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
endif()

ament_export_include_directories(include)
ament_package()
