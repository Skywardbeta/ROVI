cmake_minimum_required(VERSION 3.22)
project(rovi_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(gz_sim_vendor REQUIRED)
  find_package(gz-sim REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rovi_core 0.1 CONFIG REQUIRED)
  find_package(rovi_interfaces REQUIRED)
  find_package(rovi_policy REQUIRED)

  add_library(rovi_slice_fixture STATIC test/slice_fixture.cpp)
  target_include_directories(rovi_slice_fixture PUBLIC test)
  target_link_libraries(rovi_slice_fixture PUBLIC
    ament_index_cpp::ament_index_cpp
    rclcpp::rclcpp
    rclcpp_components::component_manager
    ${rosgraph_msgs_TARGETS}
    ${rovi_interfaces_TARGETS})
  rovi_target_warnings(rovi_slice_fixture)

  add_library(rovi_gazebo_fixture STATIC test/gazebo_fixture.cpp)
  target_link_libraries(rovi_gazebo_fixture PUBLIC
    rovi_slice_fixture
    gz-sim::gz-sim
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS})
  rovi_target_warnings(rovi_gazebo_fixture)

  ament_add_ros_isolated_gtest(rovi_vertical_slice_test test/vertical_slice_test.cpp TIMEOUT 600)
  target_link_libraries(rovi_vertical_slice_test rovi_slice_fixture)
  rovi_target_warnings(rovi_vertical_slice_test)

  ament_add_ros_isolated_gtest(rovi_policy_slice_test test/policy_slice_test.cpp TIMEOUT 300)
  set(policy_adapter "${CMAKE_CURRENT_SOURCE_DIR}/../../adapters/lerobot")
  target_compile_definitions(rovi_policy_slice_test PRIVATE ROVI_POLICY_ADAPTER="${policy_adapter}")
  target_link_libraries(rovi_policy_slice_test rovi_slice_fixture rovi_policy::rovi_policy_contract)
  rovi_target_warnings(rovi_policy_slice_test)

  string(MD5 gazebo_partition "${CMAKE_CURRENT_BINARY_DIR}")
  foreach(gazebo_test IN ITEMS gazebo_smoke arm_smoke thirty_arm_smoke so101_playback so101_fallback)
    ament_add_ros_isolated_gtest(rovi_${gazebo_test}_test
      test/${gazebo_test}_test.cpp
      ENV GZ_IP=127.0.0.1
          GZ_PARTITION=rovi_${gazebo_partition}_${gazebo_test}
          LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_SOURCE_DIR}/test/lsan_gazebo.supp
          TSAN_OPTIONS=halt_on_error=1:suppressions=${CMAKE_CURRENT_SOURCE_DIR}/test/tsan_gazebo.supp
      TIMEOUT 600)
    target_link_libraries(rovi_${gazebo_test}_test rovi_gazebo_fixture)
    rovi_target_warnings(rovi_${gazebo_test}_test)
  endforeach()
  set_tests_properties(rovi_thirty_arm_smoke_test PROPERTIES TIMEOUT 2400)
  foreach(so101_test IN ITEMS so101_playback so101_fallback)
    target_compile_definitions(rovi_${so101_test}_test PRIVATE ROVI_POLICY_ADAPTER="${policy_adapter}")
    target_link_libraries(rovi_${so101_test}_test rovi_policy::rovi_policy_contract)
  endforeach()
  add_test(NAME rovi_thirty_arm_files_current
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/generate_thirty_arms.sh --check)
  add_test(NAME rovi_manifest_provenance
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/manifest_test.sh
            ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/write_manifest.sh)
  add_test(NAME rovi_source_provenance
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/source_provenance_test.sh
            ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/print_source_provenance.sh)
  add_test(NAME rovi_comparison_provenance
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/comparison_run_test.sh
            ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/run_comparison.sh)
endif()

ament_package()
