cmake_minimum_required(VERSION 3.8)
project(TeleARM)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

#Copy these 6 folders from my source directory into the install directory, under the path share /TeleARM/
install(DIRECTORY
  launch
  urdf
  config
  meshes
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

#Copies your 6 Pythoon node files and marks each one as an executable, which makes ros2 run TeleARM <name> work 
install(PROGRAMS
  scripts/diff_drive_publisher.py
  scripts/gripper_publisher.py
  scripts/sim_box_detector.py
  scripts/mission.py
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
