cmake_minimum_required(VERSION 3.8)
project(kau_path_arbiter)

# 이 패키지는 곡선 수학을 돌리지 않아(제어점을 그대로 중계한다) -O 가
# 성능을 좌우하지 않는다. 그래도 워크스페이스의 다른 패키지와 빌드 타입을
# 맞춰 두는 편이 진단할 때 헷갈리지 않는다.
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
find_package(std_msgs REQUIRED)
find_package(kau_msgs REQUIRED)

# 중재 로직은 ROS 의존성이 없다 -- 노드 없이 gtest 로 검증한다.
add_library(kau_path_arbiter_core STATIC src/arbiter.cpp)
target_include_directories(kau_path_arbiter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(path_arbiter_node src/path_arbiter_node.cpp)
target_link_libraries(path_arbiter_node kau_path_arbiter_core)
ament_target_dependencies(path_arbiter_node rclcpp std_msgs kau_msgs)

install(TARGETS path_arbiter_node
  DESTINATION lib/${PROJECT_NAME}
)
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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_arbiter test/test_arbiter.cpp)
  if(TARGET test_arbiter)
    target_link_libraries(test_arbiter kau_path_arbiter_core)
  endif()
endif()

ament_export_include_directories(include)
ament_package()
