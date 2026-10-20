cmake_minimum_required(VERSION 3.22)
project(rovi_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ComponentSettings.msg
  msg/Decision.msg
  msg/GroupRelease.msg
  msg/JobRequest.msg
  msg/Payload.msg
  msg/RequestReceipt.msg
  msg/Reservation.msg
  msg/ReservationViolation.msg
  msg/RobotState.msg
  msg/RunInvalidation.msg
  msg/RunMetadata.msg
  msg/SchedulerCycle.msg
  msg/ServiceSource.msg
  msg/TaskProgress.msg)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
