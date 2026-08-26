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

# 역할별 모듈. 한 클래스의 멤버 구현을 여러 TU 로 나눠 담는다.
add_executable(kau_lane_detection_node
  src/kau_lane_detection_node.cpp    # 진입점 (main)

  src/lane_geometry.cpp              # 상태 없는 기하 유틸 (클래스 밖)

  src/node_setup.cpp                 # 생성자 / 파라미터 / 배선
  src/bev_geometry.cpp               # BEV 사다리꼴 / 축척 / CameraInfo
  src/camera_pose.cpp                # pan 램프 / tilt 고정 자세
  src/lane_detect.cpp                # 히스토그램 + 슬라이딩 윈도우
  src/center_resolve.cpp             # 도로 단면 판정 / 중앙선 기준
  src/path_build.cpp                 # Bezier 생성 / EMA / 게이트
  src/path_publish.cpp               # KauPath / RViz 발행
  src/pan_aim.cpp                    # Pan 조준각 (feedforward)
  src/pan_search.cpp                 # Pan 소실 탐색 상태기계
  src/map_context.cpp                # 측위 TF / 전역경로 곡률 / 장애물
  src/debug_view.cpp                 # 디버그 오버레이
  src/lane_pipeline.cpp              # 프레임 파이프라인 (imageCallback)
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
  scripts/calibrate_camera.py
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