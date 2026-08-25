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
  geometry_msgs
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
  launch/odom_reset.py
  scripts/imu_bias.py
  DESTINATION lib/${PROJECT_NAME}
)

# 지도도 설치한다. launch 가 install 트리에서 도는데 pbstream:=kau_v3 처럼
# 이름만 받으려면 지도가 share 안에 있어야 하기 때문이다.
#
# ── 왜 install(DIRECTORY) 가 아닌가 (2026-08-26) ─────────────────────
# --symlink-install 은 **폴더가 아니라 파일 하나하나에 심링크를 건다**
# (ament_cmake_symlink_install_directory 가 GLOB 로 훑어서 건다).
# 그래서 빌드 시점에 있던 지도만 심링크가 생기고, save_map.py 로
# 나중에 저장한 kau_v5 는 install 트리에 아예 없다 -> launch 의 지도
# 선택 옵션에 안 뜨고 colcon build 를 한 번 더 해야 했다.
#
# 그 대신 **폴더 자체를 소스 폴더로 가리키는 심링크 하나**로 만든다.
# 그러면 새 지도가 저장되는 즉시 install 트리에 보인다. 빌드 불필요.
#
# ★ 심링크 설치일 때만 이렇게 한다. 진짜 배포 설치(AMENT_CMAKE_SYMLINK_
#   INSTALL 이 꺼진 경우)에서 소스 트리를 가리키는 심링크를 남기면
#   소스 없는 기기에서 지도가 통째로 사라진다.
if(AMENT_CMAKE_SYMLINK_INSTALL)
  install(CODE "
    set(_maps_src \"${CMAKE_CURRENT_SOURCE_DIR}/maps\")
    set(_maps_dst \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/maps\")

    # 이전 빌드가 남긴 진짜 폴더(안에 파일별 심링크가 들어 있다)를
    # 걷어낸다. file(REMOVE_RECURSE) 는 심링크를 따라가지 않으므로
    # src/ 의 원본 지도는 안전하다 (2026-08-26 확인).
    if(IS_SYMLINK \"\${_maps_dst}\")
      file(REMOVE \"\${_maps_dst}\")
    elseif(EXISTS \"\${_maps_dst}\")
      file(REMOVE_RECURSE \"\${_maps_dst}\")
    endif()

    get_filename_component(_maps_parent \"\${_maps_dst}\" DIRECTORY)
    file(MAKE_DIRECTORY \"\${_maps_parent}\")
    file(CREATE_LINK \"\${_maps_src}\" \"\${_maps_dst}\" SYMBOLIC)
    message(STATUS \"symlinking: \${_maps_dst} -> \${_maps_src}\")
  ")
else()
  install(DIRECTORY
    maps
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
