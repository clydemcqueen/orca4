cmake_minimum_required(VERSION 3.12.2)
project(orca_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(
  base_controller
  src/base_controller.cpp
  src/underwater_motion.cpp
)

target_include_directories(base_controller PRIVATE include)

ament_auto_add_executable(
  camera_info_publisher
  src/camera_info_publisher.cpp
)

target_include_directories(camera_info_publisher PRIVATE include)

ament_auto_add_executable(
  manager
  src/manager.cpp
)

target_include_directories(manager PRIVATE include)

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install scripts
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  scripts/test_base.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()