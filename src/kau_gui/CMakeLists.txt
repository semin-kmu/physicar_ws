cmake_minimum_required(VERSION 3.8)
project(kau_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(kau_msgs REQUIRED)

# rviz2 / rqt 가 이미 의존하는 Qt5. GUI 전용이며 제어단은 링크하지 않는다.
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# Q_OBJECT 를 쓰는 header 가 include/ 아래 있으므로 target source 에 함께 넣는다.
set(CMAKE_AUTOMOC ON)

add_executable(kau_gui_node
  src/main.cpp
  src/ros_bridge.cpp
  src/map_view.cpp
  src/plot_axis.cpp
  src/plot_strip.cpp
  src/status_bar.cpp
  src/main_window.cpp
  include/kau_gui/map_view.hpp
  include/kau_gui/plot_strip.hpp
  include/kau_gui/status_bar.hpp
  include/kau_gui/main_window.hpp
)

target_include_directories(kau_gui_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(kau_gui_node
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  tf2_geometry_msgs::tf2_geometry_msgs
)

ament_target_dependencies(kau_gui_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  kau_msgs
)

install(TARGETS kau_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # uncrustify 기본 규약이 팀 코드 스타일(빈 줄 다용)과 충돌한다 (kau_control 과 동일)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
