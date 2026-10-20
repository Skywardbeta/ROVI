cmake_minimum_required(VERSION 3.22)
project(rovi_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(cmake/rovi_warnings.cmake)

add_library(rovi_core
  src/admission.cpp
  src/audit.cpp
  src/capacity_calendar.cpp
  src/clock_guard.cpp
  src/cohort.cpp
  src/comparison.cpp
  src/comparison_metrics.cpp
  src/engine.cpp
  src/interval_calendar.cpp
  src/job.cpp
  src/receipt.cpp
  src/scheduler.cpp
  src/wireless_profile.cpp
  src/window.cpp)
add_library(rovi_core::rovi_core ALIAS rovi_core)
target_include_directories(rovi_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(rovi_core PUBLIC cxx_std_20)
set_target_properties(rovi_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
rovi_target_warnings(rovi_core)

add_executable(rovi_compare tools/rovi_compare.cpp)
target_link_libraries(rovi_compare PRIVATE rovi_core)
rovi_target_warnings(rovi_compare)
install(TARGETS rovi_compare RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(TARGETS rovi_core EXPORT rovi_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT rovi_coreTargets NAMESPACE rovi_core::
  DESTINATION ${CMAKE_INSTALL_DATADIR}/rovi_core/cmake)
configure_package_config_file(cmake/rovi_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/rovi_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATADIR}/rovi_core/cmake)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/rovi_coreConfigVersion.cmake
  COMPATIBILITY SameMinorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/rovi_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/rovi_coreConfigVersion.cmake
  cmake/rovi_warnings.cmake
  DESTINATION ${CMAKE_INSTALL_DATADIR}/rovi_core/cmake)
install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/rovi_core)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest 1.14 CONFIG REQUIRED)
  include(GoogleTest)
  add_executable(rovi_core_tests
    test/admission_test.cpp
    test/capacity_calendar_test.cpp
    test/clock_guard_test.cpp
    test/cohort_test.cpp
    test/comparison_invariant_test.cpp
    test/comparison_recovery_test.cpp
    test/comparison_test.cpp
    test/digest_test.cpp
    test/engine_test.cpp
    test/interval_calendar_test.cpp
    test/job_test.cpp
    test/oracle_test.cpp
    test/receipt_test.cpp
    test/scheduler_test.cpp
    test/units_test.cpp
    test/wireless_profile_test.cpp
    test/window_test.cpp)
  target_link_libraries(rovi_core_tests PRIVATE rovi_core GTest::gtest_main)
  rovi_target_warnings(rovi_core_tests)
  gtest_discover_tests(rovi_core_tests DISCOVERY_MODE PRE_TEST)
  add_test(NAME rovi_compare_cli
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/compare_cli_test.sh $<TARGET_FILE:rovi_compare>)
  add_test(NAME rovi_core_boundary
    COMMAND ${CMAKE_COMMAND} -DROVI_CORE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_core_boundary.cmake)
endif()
