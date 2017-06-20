cmake_minimum_required(VERSION 2.8.3)
project(test_diagnostic_aggregator)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED diagnostic_aggregator diagnostic_msgs pluginlib roscpp rospy rostest)

catkin_package(DEPENDS diagnostic_aggregator diagnostic_msgs pluginlib roscpp rospy
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(test_diagnostic_aggregator
  src/match_no_analyze_analyzer.cpp
  src/fail_init_analyzer.cpp)
add_dependencies(test_diagnostic_aggregator ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_diagnostic_aggregator ${catkin_LIBRARIES})

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
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES test_diagnostic_aggregator_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
