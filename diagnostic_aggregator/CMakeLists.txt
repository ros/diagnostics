# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(diagnostic_aggregator)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED diagnostic_msgs pluginlib roscpp rospy rostest xmlrpcpp)

catkin_package(DEPENDS diagnostic_msgs pluginlib roscpp rospy xmlrpcpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})

find_package(Boost REQUIRED COMPONENTS system regex)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/status_item.cpp
  src/analyzer_group.cpp
  src/generic_analyzer.cpp
  src/discard_analyzer.cpp
  src/ignore_analyzer.cpp
  src/aggregator.cpp)
target_link_libraries(diagnostic_aggregator ${Boost_LIBRARIES}
                                            ${catkin_LIBRARIES}
)

# Aggregator node 
add_executable(aggregator_node src/aggregator_node.cpp)
target_link_libraries(aggregator_node ${catkin_LIBRARIES}
                                      ${PROJECT_NAME}
)

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/launch/test_agg.launch)

  # Analyzer loader allows other users to test that Analyzers load
  catkin_add_gtest(analyzer_loader test/analyzer_loader.cpp)
  target_link_libraries(analyzer_loader diagnostic_aggregator)
  set_target_properties(analyzer_loader PROPERTIES EXCLUDE_FROM_ALL FALSE)


  # Test Analyzer loader
  add_rostest(test/launch/test_loader.launch)
  add_rostest(test/launch/test_expected_stale.launch)
  add_rostest(test/launch/test_multiple_match.launch)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES analyzer_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS aggregator_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
