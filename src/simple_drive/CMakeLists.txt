cmake_minimum_required(VERSION 3.8)
project(simple_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(backup_msgs REQUIRED)

set(SIMPLE_DRIVE_DEPS rclcpp std_msgs backup_msgs)

add_executable(simple_decision_node
  src/decision_main.cpp
  src/decision_node.cpp
)
target_include_directories(simple_decision_node PRIVATE include)
ament_target_dependencies(simple_decision_node ${SIMPLE_DRIVE_DEPS})

add_executable(simple_control_node
  src/control_main.cpp
  src/control_node.cpp
)
target_include_directories(simple_control_node PRIVATE include)
ament_target_dependencies(simple_control_node ${SIMPLE_DRIVE_DEPS})

install(TARGETS simple_decision_node simple_control_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 이 워크스페이스는 소스에 copyright 헤더를 달지 않는다 (backup_* 전부
  # 동일). cpplint 의 legal/copyright 는 파일마다 무조건 실패하고, 그
  # 소음에 묻혀 진짜 지적이 안 보인다. cpplint 만 끄고 uncrustify /
  # pep257 등 나머지 lint 는 그대로 건다.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_simple_drive test/test_simple_drive.cpp)
  if(TARGET test_simple_drive)
    target_include_directories(test_simple_drive PRIVATE include)
    ament_target_dependencies(test_simple_drive ${SIMPLE_DRIVE_DEPS})
  endif()
endif()

ament_package()
