cmake_minimum_required(VERSION 3.22)
project(rovi_robot_agent LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rovi_core 0.1 CONFIG REQUIRED)
find_package(rovi_interfaces REQUIRED)
find_package(rovi_policy REQUIRED)
find_package(rovi_telemetry REQUIRED)

add_library(rovi_robot_agent_logic STATIC
  src/arm_commander.cpp
  src/arm_joints.cpp
  src/chunk_commander.cpp
  src/chunk_player.cpp
  src/joint_calibration.cpp
  src/joint_goal_follower.cpp
  src/messages.cpp
  src/motion_monitor.cpp
  src/robot_agent.cpp
  src/waypoint_commander.cpp
  src/waypoint_follower.cpp)
target_include_directories(rovi_robot_agent_logic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(rovi_robot_agent_logic PUBLIC rovi_core::rovi_core ${rovi_interfaces_TARGETS})
set_target_properties(rovi_robot_agent_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)
rovi_target_warnings(rovi_robot_agent_logic)

add_library(rovi_robot_agent_component SHARED
  src/arm_controller_node.cpp
  src/node_parameters.cpp
  src/robot_agent_node.cpp
  src/waypoint_controller_node.cpp)
target_link_libraries(rovi_robot_agent_component PRIVATE rovi_telemetry::rovi_telemetry
  rovi_policy::rovi_policy_contract
  rovi_robot_agent_logic rclcpp::rclcpp rclcpp_components::component
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS})
rovi_target_warnings(rovi_robot_agent_component)
rclcpp_components_register_nodes(rovi_robot_agent_component
  "rovi::robot_agent::ArmControllerNode"
  "rovi::robot_agent::RobotAgentNode"
  "rovi::robot_agent::WaypointControllerNode")
install(TARGETS rovi_robot_agent_component LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(rovi_robot_agent_tests
    test/arm_commander_test.cpp
    test/arm_joints_test.cpp
    test/chunk_commander_test.cpp
    test/chunk_player_test.cpp
    test/joint_calibration_test.cpp
    test/joint_goal_follower_test.cpp
    test/messages_test.cpp
    test/motion_monitor_test.cpp
    test/robot_agent_test.cpp
    test/waypoint_commander_test.cpp
    test/waypoint_follower_test.cpp)
  target_link_libraries(rovi_robot_agent_tests rovi_robot_agent_logic)
  rovi_target_warnings(rovi_robot_agent_tests)
endif()

ament_package()
