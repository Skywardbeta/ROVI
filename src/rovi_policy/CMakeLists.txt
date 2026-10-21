cmake_minimum_required(VERSION 3.22)
project(rovi_policy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rovi_core 0.1 CONFIG REQUIRED)
find_package(rovi_interfaces REQUIRED)
find_package(rovi_telemetry REQUIRED)

add_library(rovi_policy_contract STATIC src/wire.cpp)
target_include_directories(rovi_policy_contract PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rovi_policy_contract PUBLIC cxx_std_20)
set_target_properties(rovi_policy_contract PROPERTIES POSITION_INDEPENDENT_CODE ON)
rovi_target_warnings(rovi_policy_contract)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(rovi_policy_tests
    test/wire_test.cpp)
  target_link_libraries(rovi_policy_tests rovi_policy_contract)
  rovi_target_warnings(rovi_policy_tests)
endif()

install(TARGETS rovi_policy_contract EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
