cmake_minimum_required(VERSION 3.22)
project(litearm_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY robot_param DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Install scripts as executables
install(PROGRAMS
  scripts/aging_test.py
  scripts/joint_angle_monitor.py
  scripts/left_arm_pose5.py
  DESTINATION lib/${PROJECT_NAME}
)
