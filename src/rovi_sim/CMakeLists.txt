cmake_minimum_required(VERSION 3.22)
project(rovi_sim NONE)

find_package(ament_cmake REQUIRED)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models/rovi_panda_fuel_v5/model.sdf")
  message(WARNING "models/rovi_panda_fuel_v5 is missing: run scripts/fetch_models.sh, then rebuild")
endif()

install(DIRECTORY config models worlds DESTINATION share/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
