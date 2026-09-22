cmake_minimum_required(VERSION 3.22)
project(pixi_kuka_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(pixi_kuka_fri_controllers SHARED
  src/fri_command_controllers.cpp
)
target_include_directories(pixi_kuka_fri_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pixi_kuka_fri_controllers
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  controller_interface pixi_kuka_fri_controllers.xml
)

install(TARGETS pixi_kuka_fri_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/kuka_ext_torque_relay.py DESTINATION lib/${PROJECT_NAME})

ament_package()
