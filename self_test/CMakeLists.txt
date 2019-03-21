cmake_minimum_required(VERSION 3.5)
project(self_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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

add_executable(selftest_example src/selftest_example.cpp)
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

ament_package()
