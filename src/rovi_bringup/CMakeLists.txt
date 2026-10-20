cmake_minimum_required(VERSION 3.22)
project(rovi_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config foxglove launch DESTINATION share/${PROJECT_NAME})

ament_package()
