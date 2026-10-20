cmake_minimum_required(VERSION 3.22)
project(rovi_telemetry LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rovi_core 0.1 CONFIG REQUIRED)
find_package(rovi_interfaces REQUIRED)

add_library(rovi_telemetry INTERFACE)
target_include_directories(rovi_telemetry INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rovi_telemetry INTERFACE cxx_std_20)
target_link_libraries(rovi_telemetry INTERFACE
  rclcpp::rclcpp rovi_core::rovi_core ${rovi_interfaces_TARGETS})

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(rovi_telemetry_tests test/run_clock_test.cpp)
  target_link_libraries(rovi_telemetry_tests rovi_telemetry)
  rovi_target_warnings(rovi_telemetry_tests)
endif()

install(TARGETS rovi_telemetry EXPORT rovi_telemetryTargets)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(rovi_telemetryTargets)
ament_export_dependencies(rclcpp rovi_core rovi_interfaces)
ament_package()
