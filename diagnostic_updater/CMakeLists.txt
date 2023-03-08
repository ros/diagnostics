cmake_minimum_required(VERSION 3.5)
project(diagnostic_updater)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}
  INTERFACE
  diagnostic_msgs
  rclcpp
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(example src/example.cpp)
ament_target_dependencies(
  example
  "diagnostic_msgs"
  "rclcpp"
  "std_msgs"
)
target_link_libraries(example ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE) # skip for https://github.com/ros-tooling/action-ros-lint/issues/354
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  ament_add_gtest(diagnostic_updater_test test/diagnostic_updater_test.cpp)
  target_include_directories(diagnostic_updater_test
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(
    diagnostic_updater_test
    "diagnostic_msgs"
    "rclcpp"
    "rclcpp_lifecycle"
    "std_msgs"
  )

  ament_add_gtest(diagnostic_status_wrapper_test test/diagnostic_status_wrapper_test.cpp)
  target_include_directories(diagnostic_status_wrapper_test
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(
    diagnostic_status_wrapper_test
    "diagnostic_msgs"
    "rclcpp"
    "rclcpp_lifecycle"
    "std_msgs"
  )

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(diagnostic_updater_test.py "test/diagnostic_updater_test.py")
  ament_add_pytest_test(test_DiagnosticStatusWrapper.py "test/test_diagnostic_status_wrapper.py")
endif()

ament_python_install_package(${PROJECT_NAME})
install(
  FILES ${PROJECT_NAME}/example.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS example
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)

ament_package()
