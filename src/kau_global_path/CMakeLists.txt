cmake_minimum_required(VERSION 3.8)
project(kau_global_path)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/record_trajectory.py
  scripts/global_path_publisher.py
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
