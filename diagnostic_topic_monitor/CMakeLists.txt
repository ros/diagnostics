cmake_minimum_required(VERSION 3.8)
project(diagnostic_topic_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(std_msgs REQUIRED)

add_library(topic_monitor_component SHARED src/topic_monitor_component.cpp)
rclcpp_components_register_node(topic_monitor_component
  PLUGIN "rob_topic_monitor::TopicMonitor"
  EXECUTABLE topic_monitor
)
target_include_directories(topic_monitor_component PRIVATE include)
ament_target_dependencies(topic_monitor_component
  "lifecycle_msgs"
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_components"
  "diagnostic_msgs"
  "diagnostic_updater"
)
ament_export_targets(export_topic_monitor_component)
install(TARGETS topic_monitor_component
  EXPORT export_topic_monitor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(header_topic_monitor_component SHARED src/header_topic_monitor_component.cpp)
rclcpp_components_register_node(header_topic_monitor_component
  PLUGIN "rob_topic_monitor::HeaderTopicMonitor"
  EXECUTABLE header_topic_monitor
)
target_include_directories(header_topic_monitor_component PRIVATE include)
ament_target_dependencies(header_topic_monitor_component
  "lifecycle_msgs"
  "std_msgs"
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_components"
  "diagnostic_msgs"
  "diagnostic_updater"
)
ament_export_targets(export_header_topic_monitor_component)
install(TARGETS header_topic_monitor_component
  EXPORT export_header_topic_monitor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
# Install config files.
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  file(GLOB _pytest_tests TESTS test/*.py)
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

ament_package()
