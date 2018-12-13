cmake_minimum_required(VERSION 3.5)

project(diagnostic_aggregator)

# Support C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# Load catkin and all dependencies required for this package
#find_package(catkin REQUIRED diagnostic_msgs pluginlib roscpp rospy xmlrpcpp bondcpp)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bondcpp REQUIRED)
find_package(rclpy REQUIRED)


#catkin_package(DEPENDS diagnostic_msgs pluginlib roscpp rospy xmlrpcpp bondcpp
#    INCLUDE_DIRS include
#    LIBRARIES ${PROJECT_NAME})
# Include boost
find_package(Boost REQUIRED system)

set(INCLUDE_DIRS
  include
  ${diagnostic_msgs_INCLUDE_DIRS}
  ${builtin_interfaces_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclpy_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${bondcpp_INCLUDE_DIRS}
)

set(LIBS
  ${diagnostic_msgs_LIBRARIES}
  ${builtin_interfaces_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rclpy_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${bondcpp_LIBRARIES}
)

include_directories(${INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/status_item.cpp
  src/analyzer_group.cpp
  src/generic_analyzer.cpp
  src/discard_analyzer.cpp
  src/ignore_analyzer.cpp
  src/aggregator.cpp)
target_link_libraries(diagnostic_aggregator ${LIBS}
)

# Aggregator node
add_executable(aggregator_node src/aggregator_node.cpp)
target_link_libraries(aggregator_node ${PROJECT_NAME})
install(
  TARGETS aggregator_node
  DESTINATION lib/${PROJECT_NAME})


# Analyzer loader allows other users to test that Analyzers load
find_package(ament_cmake_gtest REQUIRED)

add_executable(analyzer_loader test/analyzer_loader.cpp)
target_link_libraries(analyzer_loader diagnostic_aggregator)
install(
  TARGETS analyzer_loader
  DESTINATION lib/${PROJECT_NAME})

add_executable(multi_match_pub test/multi_match_pub.cpp)
target_link_libraries(multi_match_pub ${LIBS})
install(
  TARGETS multi_match_pub
  DESTINATION lib/${PROJECT_NAME})

add_executable(expected_stale_pub test/expected_stale_pub.cpp)
target_link_libraries(expected_stale_pub ${LIBS})
install(
  TARGETS expected_stale_pub
  DESTINATION lib/${PROJECT_NAME})

add_executable(diag_pub test/diag_pub.cpp)
target_link_libraries(diag_pub ${LIBS})
install(
  TARGETS diag_pub
  DESTINATION lib/${PROJECT_NAME})
add_executable(add_analyze_pub test/add_analyze_pub.cpp)
target_link_libraries(add_analyze_pub ${LIBS})
install(
  TARGETS add_analyze_pub
  DESTINATION lib/${PROJECT_NAME})

add_executable(aggregator_test_pub  test/aggregator_test_pub.cpp)
target_link_libraries(aggregator_test_pub  ${LIBS})
install(
  TARGETS aggregator_test_pub
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  # add_rostest(test/launch/test_agg.launch)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  #  Below test cases will faile due to bug  LaunchService.shutdown() fails to terminate run loop
  #  ament_add_pytest_test(aggregator_test.py  "test/aggregator_test.py")
  #  ament_add_pytest_test(add_analyzers_test.py  "test/add_analyzers_test.py")
  #  ament_add_pytest_test(expected_stale_test.py  "test/expected_stale_test.py")
  #  ament_add_pytest_test(multiple_match_test.py  "test/multiple_match_test.py")

endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE "DAIGNOSTICCPP_BUILDING_DLL")
############################################################
# Define installation rules

install(
  TARGETS ${PROJECT_NAME} aggregator_node analyzer_loader
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  FILES analyzer_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export include dependencies, directories, and libraries
ament_export_dependencies(ament_cmake)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(${PROJECT_NAME})
ament_export_dependencies(pluginlib)
ament_export_include_directories(${INCLUDE_DIRS})
pluginlib_export_plugin_description_file(${PROJECT_NAME} analyzer_plugins.xml)


# Must be at the bottom
ament_package()
