cmake_minimum_required(VERSION 3.8)
project(TeleARM)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  config
  meshes
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/diff_drive_publisher.py
  scripts/gripper_publisher.py
  TeleARM_control/cleanup_controller.py
  TeleARM_control/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
