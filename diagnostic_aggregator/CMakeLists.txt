# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.0.2)
project(diagnostic_aggregator)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED diagnostic_msgs pluginlib roscpp rospy xmlrpcpp bondcpp)
catkin_package(DEPENDS diagnostic_msgs pluginlib roscpp rospy xmlrpcpp bondcpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})

set(REQUIRED_diagnostic_msgs_VERSION_Indigo "1.11.9")
set(REQUIRED_diagnostic_msgs_VERSION_Jade "1.12.4")
if( ${diagnostic_msgs_VERSION} VERSION_LESS ${REQUIRED_diagnostic_msgs_VERSION_Indigo})
  message(FATAL_ERROR "diagnostic_msgs version ${REQUIRED_diagnostic_msgs_VERSION_Indigo} or newer is required to build diagnotic_aggregator on ROS Indigo")
endif()
if( (${diagnostic_msgs_VERSION} VERSION_EQUAL "1.12.0") OR
    (${diagnostic_msgs_VERSION} VERSION_GREATER "1.12.0" AND
     ${diagnostic_msgs_VERSION} VERSION_LESS ${REQUIRED_diagnostic_msgs_VERSION_Jade}))
  message(FATAL_ERROR "diagnostic_msgs version ${REQUIRED_diagnostic_msgs_VERSION_Jade} or newer is required to build diagnotic_aggregator on ROS Jade")
endif()

find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

set(LOCAL_GTEST_DIR "gtest-1.7.0")
# gtest could be included in ${catkin_INCLUDE_DIRS}, prepend local gtest include directory so it's searched first
include_directories(BEFORE ${LOCAL_GTEST_DIR}/include)

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
add_dependencies(${PROJECT_NAME} diagnostic_msgs_generate_messages_cpp)

# Aggregator node 
add_executable(aggregator_node src/aggregator_node.cpp)
target_link_libraries(aggregator_node ${catkin_LIBRARIES}
                                      ${PROJECT_NAME}
)

# Analyzer loader allows other users to test that Analyzers load
add_executable(analyzer_loader test/analyzer_loader.cpp ${LOCAL_GTEST_DIR}/gtest-all.cc)
target_link_libraries(analyzer_loader diagnostic_aggregator)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/launch/test_agg.launch)
  add_rostest(test/launch/test_add_agg.launch)

  # Test Analyzer loader
  add_rostest(test/launch/test_loader.launch)
  add_rostest(test/launch/test_expected_stale.launch)
  add_rostest(test/launch/test_multiple_match.launch)

  add_rostest(test/launch/test_discard_stale_not_published.launch)
endif()

catkin_install_python(
        PROGRAMS scripts/add_analyzers
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES analyzer_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS aggregator_node analyzer_loader
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
