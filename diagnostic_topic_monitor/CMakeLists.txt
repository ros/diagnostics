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

add_library(topic_frequency_monitor_component SHARED src/topic_frequency_monitor_component.cpp)
rclcpp_components_register_node(topic_frequency_monitor_component
  PLUGIN "diagnostic_topic_monitor::TopicFrequencyMonitor"
  EXECUTABLE topic_frequency_monitor
)
target_include_directories(topic_frequency_monitor_component PRIVATE include)
target_link_libraries(topic_frequency_monitor_component PUBLIC
  ${diagnostic_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  rclcpp_lifecycle::rclcpp_lifecycle
)
ament_export_targets(export_topic_frequency_monitor_component)
install(TARGETS topic_frequency_monitor_component
  EXPORT export_topic_frequency_monitor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(topic_age_monitor_component SHARED src/topic_age_monitor_component.cpp)
rclcpp_components_register_node(topic_age_monitor_component
  PLUGIN "diagnostic_topic_monitor::TopicAgeMonitor"
  EXECUTABLE topic_age_monitor
)
target_include_directories(topic_age_monitor_component PRIVATE include)
target_link_libraries(topic_age_monitor_component PUBLIC
  ${diagnostic_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  rclcpp_lifecycle::rclcpp_lifecycle
)
ament_export_targets(export_topic_age_monitor_component)
install(TARGETS topic_age_monitor_component
  EXPORT export_topic_age_monitor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Install the test configs and dummy publisher used for testing
  install(
    DIRECTORY test
    DESTINATION share/${PROJECT_NAME}/
  )
  install(
    PROGRAMS test/dummy_publishers.py
    DESTINATION lib/${PROJECT_NAME}
  )

  add_launch_test(
    test/test_topic_frequency_monitor.py
    TARGET test_topic_frequency_monitor
    TIMEOUT 20)

  add_launch_test(
    test/test_topic_age_monitor.py
    TARGET test_topic_age_monitor
    TIMEOUT 20)
endif()

ament_package()
