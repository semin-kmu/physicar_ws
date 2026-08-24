cmake_minimum_required(VERSION 3.8)
project(kau_local_path_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kau_msgs REQUIRED)
find_package(kau_control REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 알고리즘 구현 (ROS 의존성 없음) 을 실제 라이브러리로 묶는다 -- node 와
# gtest 실행파일이 둘 다 같은 컴파일 결과물을 링크해야 하므로, 소스를
# add_executable 안에 직접 넣으면 (이전 실수) 실행파일끼리 심볼을
# 공유하지 못해 테스트 링크가 전부 실패한다.
add_library(kau_local_path_planner_impl STATIC
  src/reference_fusion.cpp
  src/candidate_generator.cpp
  src/path_evaluator.cpp
  src/collision_checker.cpp
  src/boundary_checker.cpp
  src/local_planner.cpp
)
target_include_directories(kau_local_path_planner_impl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kau_local_path_planner_impl PUBLIC Eigen3::Eigen)
ament_target_dependencies(kau_local_path_planner_impl PUBLIC
  kau_msgs
  kau_control
)

add_executable(local_planner_node src/local_planner_node.cpp)
target_link_libraries(local_planner_node
  kau_local_path_planner_impl
  tf2_geometry_msgs::tf2_geometry_msgs
  ${YAML_CPP_LIBRARIES}
)
target_include_directories(local_planner_node PRIVATE ${YAML_CPP_INCLUDE_DIR})
ament_target_dependencies(local_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  kau_msgs
  kau_control
)

install(TARGETS local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/fake_upstream.py scripts/rejection_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # cpplint 는 git 저장소 안에서만 동작하고, uncrustify 기본 규약이 팀
  # 코드 스타일(빈 줄 다용)과 충돌한다 (kau_control/CMakeLists.txt 동일 처리).
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(kau_local_path_planner_test_names
    test_candidate_generator
    test_collision_checker
    test_boundary_checker
    test_path_evaluator
    test_local_planner_scenarios
  )
  foreach(test_name ${kau_local_path_planner_test_names})
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    if(TARGET ${test_name})
      target_link_libraries(${test_name} kau_local_path_planner_impl)
      ament_target_dependencies(${test_name} kau_control kau_msgs)
    endif()
  endforeach()
endif()

ament_export_include_directories(include)
ament_package()
