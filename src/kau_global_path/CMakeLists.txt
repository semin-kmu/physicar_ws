cmake_minimum_required(VERSION 3.8)
project(kau_global_path)

# CMake 의 기본 빌드 타입은 "빈 문자열" 이고, 그러면 GCC 에 -O 가
# 아예 안 붙는다 (= -O0).
# 경로 적합/샘플링이 -O0 에서 몇 배로 느려진다.
# 2026-08-25: build/ 를 훑어보니 이 패키지가 실제로 그 상태였다
# (kau_control/kau_lane_detection 만 Release, 나머지는 빈 값).
# --cmake-args 없이 `colcon build` 해도 안전하도록 기본값을 박아 둔다.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/record_trajectory.py
  scripts/global_path_publisher.py
  scripts/global_avoidance_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# serve_editor.py / check_path.py / extract_sim_track.py 는 일부러 뺐다.
# 워크스페이스 소스 트리를 기준으로 경로를 잡는 개발용 도구라 install 트리에서는
# 자기 위치를 못 찾는다. `python3 src/.../xxx.py` 로 직접 돌린다.

install(DIRECTORY
  web
  data
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
