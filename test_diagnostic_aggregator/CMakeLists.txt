cmake_minimum_required(VERSION 2.8.3)
project(test_diagnostic_aggregator)

# Support C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Load catkin and all dependencies required for this package
#find_package(catkin REQUIRED diagnostic_aggregator diagnostic_msgs pluginlib roscpp rospy rostest)
find_package(diagnostic_aggregator REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rospy REQUIRED)
find_package(rostest REQUIRED)

# catkin_package(DEPENDS diagnostic_aggregator diagnostic_msgs pluginlib roscpp rospy
#               INCLUDE_DIRS include
#               LIBRARIES ${PROJECT_NAME}
#)

include_directories(SYSTEM ${diagnostic_aggregator_INCLUDE_DIRS} ${diagnostic_msgs_INCLUDE_DIRS} ${pluginlib_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${rostest_INCLUDE_DIRS})
include_directories(include)

add_library(test_diagnostic_aggregator
  src/match_no_analyze_analyzer.cpp
  src/fail_init_analyzer.cpp)
add_dependencies(test_diagnostic_aggregator ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_diagnostic_aggregator ${diagnostic_aggregator_LIBRARIES} ${diagnostic_msgs_LIBRARIES} ${pluginlib_LIBRARIES} ${rclcpp_LIBRARIES} ${rostest_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/launch/test_match_no_analyze_analyzer_load.launch)

  # Tests that an item can be matched, not analyzed, and reported in "Other"
  add_rostest(test/launch/test_match_no_analyze.launch)

  # Tests that item can be matched, not analyzed, but analyzed by another Analyzer
  add_rostest(test/launch/test_match_then_analyze.launch)

  # Tests that analyzers that fail to load will report an error in the diagnostics
  add_rostest(test/launch/test_fail_init.launch)

  # Tests that analyzers specified both with and without package name load correctly
  add_rostest(test/launch/test_dual_analyzer_load.launch)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include
)
install(FILES test_diagnostic_aggregator_plugin.xml
        DESTINATION share
)

ament_package()
