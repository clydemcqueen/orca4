cmake_minimum_required(VERSION 3.12.2)
project(orca_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Debugging: set _dump_all_variables to true
set(_dump_all_variables false)
if(_dump_all_variables)
  get_cmake_property(_variable_names VARIABLES)
  list(SORT _variable_names)
  foreach(_variable_name ${_variable_names})
    message(STATUS "${_variable_name}=${${_variable_name}}")
  endforeach()
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Generate ROS messages
rosidl_generate_interfaces(
  orca_msgs
  action/TargetMode.action
  msg/Effort.msg
  msg/Motion.msg
  DEPENDENCIES geometry_msgs std_msgs
)

ament_auto_package()