cmake_minimum_required(VERSION 3.8)
project(litearm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(litearm_robot REQUIRED)
find_package(pinocchio REQUIRED)

# Add hardware interface library
add_library(
  litearm_hardware
  SHARED
  src/litearm_hardware_interface.cpp
)

target_include_directories(
  litearm_hardware
  PRIVATE
  include
)

ament_target_dependencies(
  litearm_hardware
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  pinocchio
)

target_link_libraries(
  litearm_hardware
  litearm_robot::litearm_robot
  pinocchio::pinocchio
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface litearm_hardware.xml)

# Install
install(
  TARGETS litearm_hardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(
  include
)
ament_export_libraries(
  litearm_hardware
)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  litearm_robot
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()