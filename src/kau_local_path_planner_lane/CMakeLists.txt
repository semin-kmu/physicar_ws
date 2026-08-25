cmake_minimum_required(VERSION 3.8)
project(kau_local_path_planner_lane)

# CMake 의 기본 빌드 타입은 "빈 문자열" 이고, 그러면 GCC 에 -O 가
# 아예 안 붙는다 (= -O0).
# 이 패키지의 hot path 는 Eigen 동반행렬 고유값 분해(최근접점 9 차 근,
# 곡률 14 차 근)이고, 도로 판정이 후보 1 개당 그걸 1,200 회 넘게 돈다.
# -O0 과 -O3 을 같은 소스로 실측하니 roadReportWheels(300cm) 가
# 2285.9ms 대 21.0ms (109 배) 였다 -- 5Hz 예산 200ms 를 두 자릿수 배로
# 넘긴다. 빌드 타입이 안 잡히면 플래너가 아예 못 돈다.
# 2026-08-25: build/ 를 훑어보니 이 패키지가 실제로 그 상태였다
# (kau_control/kau_lane_detection 만 Release, 나머지는 빈 값).
# --cmake-args 없이 `colcon build` 해도 안전하도록 기본값을 박아 둔다.
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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kau_msgs REQUIRED)
find_package(kau_control REQUIRED)
find_package(Eigen3 REQUIRED)

# 알고리즘 구현 (ROS 의존성 없음) 을 실제 라이브러리로 묶는다 -- node 와
# gtest 실행파일이 둘 다 같은 컴파일 결과물을 링크해야 하므로, 소스를
# add_executable 안에 직접 넣으면 (이전 실수) 실행파일끼리 심볼을
# 공유하지 못해 테스트 링크가 전부 실패한다.
add_library(kau_local_path_planner_lane_impl STATIC
  src/lane_backbone.cpp
  src/candidate_generator.cpp
  src/path_evaluator.cpp
  src/collision_checker.cpp
  src/boundary_checker.cpp
  src/local_planner.cpp
)
target_include_directories(kau_local_path_planner_lane_impl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kau_local_path_planner_lane_impl PUBLIC Eigen3::Eigen)
ament_target_dependencies(kau_local_path_planner_lane_impl PUBLIC
  kau_msgs
  kau_control
)

add_executable(local_planner_node src/local_planner_node.cpp)
target_link_libraries(local_planner_node
  kau_local_path_planner_lane_impl
  tf2_geometry_msgs::tf2_geometry_msgs
)
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

# 2026-08-25 (kau_local_path_planner 에서 분기, lane-only 변형): 복사해 온
# gtest 들은 옛 global+lane fusion/map 기반 boundary 를 검증하던 테스트라
# 이번 재설계(backbone/road boundary 전면 교체)와 안 맞아 전부 비활성화한다
# -- 지금 상태로 빌드하면 컴파일도 안 된다(옛 API 호출). 실차 검증 우선
# 순위상 이번엔 새로 안 짬 -- 다음 세션에서 새 API 기준으로 재작성할 것.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # 2026-08-25: 위 주석대로 복사해 온 옛 gtest 들은 아직 꺼둔 채로 두고,
  # 새 API(좌/우 edge Curve) 기준으로 다시 쓴 것부터 하나씩 켠다.
  # test_boundary_checker_lane 은 "도로 이탈 판정이 통째로 무력했던" 회귀
  # (marginAlongNormal 의 하한 clamp) 를 잡아 두려고 만든 것이다.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_boundary_checker_lane test/test_boundary_checker_lane.cpp)
  if(TARGET test_boundary_checker_lane)
    target_link_libraries(test_boundary_checker_lane kau_local_path_planner_lane_impl)
    ament_target_dependencies(test_boundary_checker_lane kau_control kau_msgs)
  endif()

  # test_lane_backbone 은 extendCurve(등곡률 외삽)가 호길이를 현 길이로
  # 쓰던 회귀를 잡아 둔다 -- 그것 때문에 코너에서 backbone 곡률이 차량
  # 한계를 넘어 전 후보가 kappa_bound 로 탈락했다 (2026-08-25).
  ament_add_gtest(test_lane_backbone test/test_lane_backbone.cpp)
  if(TARGET test_lane_backbone)
    target_link_libraries(test_lane_backbone kau_local_path_planner_lane_impl)
    ament_target_dependencies(test_lane_backbone kau_control kau_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_package()
