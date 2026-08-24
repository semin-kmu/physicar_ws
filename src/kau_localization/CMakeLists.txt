cmake_minimum_required(VERSION 3.8)
project(kau_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# SLAM/localization 본체는 cartographer_ros 가 제공한다.
# 이 패키지가 직접 빌드하는 것은 초기 위치 릴레이 노드 하나뿐.
add_executable(initial_pose_relay src/initial_pose_relay.cpp)
ament_target_dependencies(initial_pose_relay
  rclcpp
  cartographer_ros_msgs
  geometry_msgs
)

# AMCL 이 정지 중에는 필터를 안 돌려서 map -> odom 이 얼어붙는다.
# 이 노드가 정지를 감지해 /request_nomotion_update 를 대신 호출한다.
add_executable(nomotion_updater src/nomotion_updater.cpp)
ament_target_dependencies(nomotion_updater
  rclcpp
  nav_msgs
  std_srvs
)

install(TARGETS
  initial_pose_relay
  nomotion_updater
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# 지도도 설치한다. launch 가 install 트리에서 도는데 pbstream:=kau_v3 처럼
# 이름만 받으려면 지도가 share 안에 있어야 하기 때문이다.
# 워크스페이스가 --symlink-install 이라 여기 설치되는 것은 소스를 가리키는
# 심링크다. 다만 save_map.py 로 새로 저장한 지도는 심링크가 아직 없으므로
# colcon build 를 한 번 해야 launch 가 본다 (이 패키지는 1초쯤 걸린다).
# 급하면 maps_dir:=src/kau_localization/maps 로 소스 폴더를 직접 가리킨다.
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
