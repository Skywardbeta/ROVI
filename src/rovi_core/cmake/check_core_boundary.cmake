# Fails when rovi_core library sources use floating point, clocks, sleeping, threads, randomness, I/O, the environment, or ROS/Gazebo/DDS headers.
if(NOT DEFINED ROVI_CORE_DIR)
  message(FATAL_ERROR "ROVI_CORE_DIR must name the rovi_core package directory")
endif()

file(GLOB_RECURSE core_files "${ROVI_CORE_DIR}/include/*" "${ROVI_CORE_DIR}/src/*")
if(NOT core_files)
  message(FATAL_ERROR "no rovi_core sources found under ${ROVI_CORE_DIR}")
endif()

set(forbidden_word "(^|[^A-Za-z0-9_])(float|double|getenv|system_clock|steady_clock|high_resolution_clock|rand|srand)([^A-Za-z0-9_]|$)")
set(forbidden_std "std::(atomic|mutex|shared_mutex|this_thread|thread|jthread|async|chrono|filesystem|random_device|mt19937)([^A-Za-z0-9_]|$)")
set(forbidden_call "(^|[^A-Za-z0-9_.])(time|clock|sleep|usleep)[ \t]*[(]")
set(forbidden_literal "(^|[^A-Za-z0-9_.])[0-9]+[.][0-9]*([eE][-+]?[0-9]+)?[fFlL]?([^A-Za-z0-9_]|$)|(^|[^A-Za-z0-9_.])[0-9]+[eE][-+]?[0-9]+")
set(forbidden_header "#[ \t]*include[ \t]*[<\"](chrono|ctime|time[.]h|thread|pthread[.]h|mutex|atomic|future|condition_variable|random|filesystem|fstream|iostream|cstdio|stdio[.]h|cstdlib|stdlib[.]h|unistd[.]h|cmath|math[.]h|cfloat|float[.]h|rclcpp|rcl|rmw|gz|ignition|fastdds|fastrtps|builtin_interfaces)[/.>\"]")

set(violations "")
foreach(core_file IN LISTS core_files)
  file(STRINGS "${core_file}" matches
       REGEX "${forbidden_word}|${forbidden_std}|${forbidden_call}|${forbidden_literal}|${forbidden_header}")
  foreach(match IN LISTS matches)
    list(APPEND violations "${core_file}: ${match}")
  endforeach()
endforeach()

if(violations)
  list(JOIN violations "\n" report)
  message(FATAL_ERROR "rovi_core boundary violations:\n${report}")
endif()

list(LENGTH core_files checked)
message(STATUS "rovi_core boundary check passed for ${checked} files")
