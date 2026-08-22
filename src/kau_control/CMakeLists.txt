cmake_minimum_required(VERSION 3.8)
project(kau_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kau_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# bezier.hpp / curve.hpp / pure_pursuit.hpp 는 header-only.
# Eigen 은 bezier.hpp 의 동반행렬 고유값(8.2)에 쓰인다.
add_library(kau_control_core INTERFACE)
target_include_directories(kau_control_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kau_control_core INTERFACE Eigen3::Eigen)

# 노드는 둘뿐이다. speed_controller 가 /speed, steer_controller 가 /steering.
# 공통부(경로 수신 · pose · 최근접점 추적)는 path_tracker.hpp 가 갖고 있고
# 두 노드가 각자 하나씩 인스턴스화한다. 서로를 구독하지 않는다.
foreach(node speed_controller steer_controller)
  add_executable(${node}_node src/${node}_node.cpp)

  # tf2::getYaw 는 tf2_geometry_msgs 의 fromMsg 를 끌어온다.
  # ament_target_dependencies 만으로는 링크가 안 되므로 target 을 명시한다.
  target_link_libraries(${node}_node
    kau_control_core
    tf2_geometry_msgs::tf2_geometry_msgs
  )
  ament_target_dependencies(${node}_node
    rclcpp
    geometry_msgs
    nav_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    kau_msgs
  )

  install(TARGETS ${node}_node
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/fake_path.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # cpplint only works in a git repo; this package is not one yet
  set(ament_cmake_cpplint_FOUND TRUE)
  # uncrustify 의 기본 규약이 팀 코드 스타일(빈 줄 다용)과 충돌한다
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # 레퍼런스(sim_common/curve.py)와 같은 입력에 같은 값이 나오는지 대조
  ament_add_gtest(test_curve test/test_curve.cpp)
  if(TARGET test_curve)
    target_link_libraries(test_curve kau_control_core)
  endif()
endif()

ament_export_include_directories(include)
ament_package()
