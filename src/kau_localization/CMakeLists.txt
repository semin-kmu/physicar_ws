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
find_package(sensor_msgs REQUIRED)

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

# robot_localization 에는 센서별 신뢰도 파라미터가 없다 -- 신뢰도는 오직
# 메시지의 covariance 에서 온다. 그런데 laser_odom 도 physicar_driver 도
# 공분산을 안 채워서 전부 0 으로 나가고, 그러면 EKF 가 두 소스를 모두
# "오차 0" 으로 무한신뢰한다. 이 노드가 공분산을 채워 신뢰도 손잡이를
# 만든다 (자세한 근거는 소스 상단 주석).
add_executable(odom_covariance_relay src/odom_covariance_relay.cpp)
ament_target_dependencies(odom_covariance_relay
  rclcpp
  nav_msgs
  sensor_msgs
)

install(TARGETS
  initial_pose_relay
  nomotion_updater
  odom_covariance_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# 감시자는 share/launch 에도 설치되지만 (ekf.launch.py 가 거기서 부른다),
# kau_state_machine 의 bringup manifest 는 lib/<pkg>/<exe> 만 실행한다
# (ros/process.py resolve_exe). 단계 1 에서 쓰려면 lib/ 에도 있어야 한다.
install(PROGRAMS
  launch/platform_ekf_pause.py
  launch/clock_gate.py
  DESTINATION lib/${PROJECT_NAME}
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
