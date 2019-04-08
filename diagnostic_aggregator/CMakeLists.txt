cmake_minimum_required(VERSION 3.5)
project(diagnostic_aggregator)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME}
  src/status_item.cpp
  src/analyzer_group.cpp
  src/generic_analyzer.cpp
  src/discard_analyzer.cpp
  src/ignore_analyzer.cpp
  src/aggregator.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  "diagnostic_msgs"
  "rclcpp"
  "std_msgs"
  "pluginlib"
)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  #ament_add_gtest (diagnostic_aggregator_test test/diagnostic_aggregator_test.cpp)
  #target_include_directories (diagnostic_aggregator_test
  #  PUBLIC
  #  $ <BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #  $ <INSTALL_INTERFACE:include>
  #)
  #ament_target_dependencies (
  #  diagnostic_aggregator_test
  #  "diagnostic_msgs"
  #  "rclcpp"
  #  "std_msgs"
  #)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(diagnostic_aggregator_test.py "test/add_analyzers_test.py")
endif()

ament_python_install_package(${PROJECT_NAME})
install(
  FILES ${PROJECT_NAME}/add_analyzers.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)

ament_package()
