cmake_minimum_required(VERSION 3.5)
project(self_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined.")
endif()

message(STATUS "Build for $ENV{ROS_DISTRO}")

if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNO_FUTURE")
elseif("$ENV{ROS_DISTRO}" STREQUAL "galactic")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNO_FUTURE")

  # elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
  # -- define something here if needed
  # elseif("$ENV{ROS_DISTRO}" STREQUAL "rolling")
  # -- define something here if needed
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(run_selftest src/run_selftest.cpp)
target_include_directories(run_selftest
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  run_selftest
  "builtin_interfaces"
  "diagnostic_msgs"
  "diagnostic_updater"
  "rclcpp"
  "rcutils"
)

add_executable(selftest_example example/selftest_example.cpp)
target_include_directories(selftest_example
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  selftest_example
  "builtin_interfaces"
  "diagnostic_msgs"
  "diagnostic_updater"
  "rclcpp"
  "rcutils"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS run_selftest selftest_example
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
