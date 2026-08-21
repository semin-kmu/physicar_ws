cmake_minimum_required(VERSION 3.8)
project(kau_global_path)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/record_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  web
  data
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
