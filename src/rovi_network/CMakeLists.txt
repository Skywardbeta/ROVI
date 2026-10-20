cmake_minimum_required(VERSION 3.22)
project(rovi_network LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rovi_core 0.1 CONFIG REQUIRED)
find_package(rovi_interfaces REQUIRED)
find_package(rovi_telemetry REQUIRED)

add_library(rovi_network_logic INTERFACE)
target_include_directories(rovi_network_logic INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(rovi_network_logic INTERFACE rovi_core::rovi_core)

add_library(rovi_network_component SHARED src/link_node.cpp)
target_link_libraries(rovi_network_component PRIVATE rovi_telemetry::rovi_telemetry
  rovi_network_logic rclcpp::rclcpp rclcpp_components::component ${rovi_interfaces_TARGETS})
rovi_target_warnings(rovi_network_component)
rclcpp_components_register_nodes(rovi_network_component "rovi::network::LinkNode")
install(TARGETS rovi_network_component LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(rovi_network_tests test/logical_link_test.cpp)
  target_link_libraries(rovi_network_tests rovi_network_logic)
  rovi_target_warnings(rovi_network_tests)
endif()

ament_package()
