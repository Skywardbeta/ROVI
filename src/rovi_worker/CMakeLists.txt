cmake_minimum_required(VERSION 3.22)
project(rovi_worker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rovi_core 0.1 CONFIG REQUIRED)
find_package(rovi_interfaces REQUIRED)
find_package(rovi_telemetry REQUIRED)

add_library(rovi_worker_logic STATIC src/worker_sim.cpp)
target_include_directories(rovi_worker_logic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(rovi_worker_logic PUBLIC rovi_core::rovi_core)
set_target_properties(rovi_worker_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)
rovi_target_warnings(rovi_worker_logic)

add_library(rovi_worker_component SHARED src/worker_node.cpp)
target_link_libraries(rovi_worker_component PRIVATE rovi_telemetry::rovi_telemetry
  rovi_worker_logic rclcpp::rclcpp rclcpp_components::component ${rovi_interfaces_TARGETS})
rovi_target_warnings(rovi_worker_component)
rclcpp_components_register_nodes(rovi_worker_component "rovi::worker::WorkerNode")
install(TARGETS rovi_worker_component LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(rovi_worker_tests test/worker_sim_test.cpp)
  target_link_libraries(rovi_worker_tests rovi_worker_logic)
  rovi_target_warnings(rovi_worker_tests)
endif()

ament_package()
