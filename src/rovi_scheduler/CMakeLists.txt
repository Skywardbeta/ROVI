cmake_minimum_required(VERSION 3.22)
project(rovi_scheduler LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rovi_core 0.1 CONFIG REQUIRED)
find_package(rovi_interfaces REQUIRED)
find_package(rovi_telemetry REQUIRED)

add_library(rovi_scheduler_logic STATIC
  src/config.cpp
  src/conversions.cpp
  src/job_book.cpp)
target_include_directories(rovi_scheduler_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(rovi_scheduler_logic PUBLIC rovi_core::rovi_core ${rovi_interfaces_TARGETS})
set_target_properties(rovi_scheduler_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)
rovi_target_warnings(rovi_scheduler_logic)

add_library(rovi_scheduler_component SHARED src/scheduler_node.cpp)
target_link_libraries(rovi_scheduler_component PRIVATE rovi_telemetry::rovi_telemetry
  rovi_scheduler_logic rclcpp::rclcpp rclcpp_components::component)
rovi_target_warnings(rovi_scheduler_component)
rclcpp_components_register_nodes(rovi_scheduler_component "rovi::scheduler::SchedulerNode")
install(TARGETS rovi_scheduler_component LIBRARY DESTINATION lib)
install(TARGETS rovi_scheduler_logic EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rovi_core rovi_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(rovi_scheduler_tests
    test/config_test.cpp
    test/conversions_test.cpp
    test/job_book_test.cpp)
  target_link_libraries(rovi_scheduler_tests rovi_scheduler_logic)
  rovi_target_warnings(rovi_scheduler_tests)
endif()

ament_package()
