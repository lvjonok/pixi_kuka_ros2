cmake_minimum_required(VERSION 3.22)
project(pixi_kuka_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
# pinocchio 3.9 still goes through hpp-fcl's compatibility package, which warns on every use.
set(COAL_DISABLE_HPP_FCL_WARNINGS ON)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

add_library(pixi_kuka_fri_controllers SHARED
  src/fri_command_controllers.cpp
  src/position_source.cpp
)
target_include_directories(pixi_kuka_fri_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# The same for its headers.
target_compile_definitions(pixi_kuka_fri_controllers PRIVATE COAL_DISABLE_HPP_FCL_WARNINGS)
target_link_libraries(pixi_kuka_fri_controllers
  pinocchio::pinocchio
  realtime_tools::realtime_tools
)
ament_target_dependencies(pixi_kuka_fri_controllers
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The arm the cell runs: lbr_ros2_control's iiwa14.xacro, the robot_description of
  # crisp_mock/crisp_hardware with tool:=none (its limits are the description package's minus
  # lbr_ros2_control's margin). Generated at configure time, so a broken xacro fails the build.
  find_program(XACRO_EXECUTABLE xacro REQUIRED)
  find_file(PIXI_KUKA_IIWA14_XACRO iiwa14.xacro
    PATHS ENV AMENT_PREFIX_PATH
    PATH_SUFFIXES share/lbr_ros2_control/system_integration/iiwa14
    NO_DEFAULT_PATH
    REQUIRED)
  set(test_urdf ${CMAKE_CURRENT_BINARY_DIR}/test_iiwa14.urdf)
  execute_process(
    COMMAND ${XACRO_EXECUTABLE} ${PIXI_KUKA_IIWA14_XACRO} robot_name:=lbr mode:=mock
    OUTPUT_FILE ${test_urdf}
    RESULT_VARIABLE xacro_result)
  if(NOT xacro_result EQUAL 0)
    message(FATAL_ERROR "xacro ${PIXI_KUKA_IIWA14_XACRO} failed (${xacro_result})")
  endif()

  # The core is compiled again into the test with EIGEN_RUNTIME_NO_MALLOC, so a test can assert
  # that update() never allocates.
  ament_add_gtest(test_position_source
    test/test_position_source.cpp
    src/position_source.cpp
    TIMEOUT 120)
  target_include_directories(test_position_source PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_definitions(test_position_source PRIVATE
    COAL_DISABLE_HPP_FCL_WARNINGS
    EIGEN_RUNTIME_NO_MALLOC
    PIXI_KUKA_TEST_URDF="${test_urdf}")
  # Eigen's no-malloc check is an assert; keep it (and Eigen's size checks) in a Release build.
  target_compile_options(test_position_source PRIVATE -UNDEBUG)
  target_link_libraries(test_position_source pinocchio::pinocchio)
endif()

ament_package()
