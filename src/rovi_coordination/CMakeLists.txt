cmake_minimum_required(VERSION 3.22)
project(rovi_coordination LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rovi_core 0.1 CONFIG REQUIRED)
find_package(rovi_interfaces REQUIRED)
find_package(rovi_telemetry REQUIRED)
find_package(rovi_scheduler REQUIRED)

add_library(rovi_coordination_logic STATIC
  src/group_assembler.cpp
  src/group_releaser.cpp)
target_include_directories(rovi_coordination_logic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(rovi_coordination_logic PUBLIC
  rovi_core::rovi_core rovi_scheduler::rovi_scheduler_logic)
set_target_properties(rovi_coordination_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)
rovi_target_warnings(rovi_coordination_logic)

add_library(rovi_coordination_component SHARED src/coordinator_node.cpp)
target_link_libraries(rovi_coordination_component PRIVATE rovi_telemetry::rovi_telemetry
  rovi_coordination_logic rovi_scheduler::rovi_scheduler_logic rclcpp::rclcpp
  rclcpp_components::component ${rovi_interfaces_TARGETS})
rovi_target_warnings(rovi_coordination_component)
rclcpp_components_register_nodes(rovi_coordination_component "rovi::coordination::CoordinatorNode")
install(TARGETS rovi_coordination_component LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(rovi_coordination_tests
    test/group_assembler_test.cpp
    test/group_releaser_test.cpp)
  target_include_directories(rovi_coordination_tests PRIVATE test)
  target_link_libraries(rovi_coordination_tests rovi_coordination_logic)
  rovi_target_warnings(rovi_coordination_tests)
endif()

ament_package()
