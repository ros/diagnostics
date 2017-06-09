# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(diagnostic_aggregator)

# Support C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Load catkin and all dependencies required for this package
#find_package(catkin REQUIRED diagnostic_msgs pluginlib roscpp rospy xmlrpcpp bondcpp)
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rospy REQUIRED)
find_package(xmlrpcpp REQUIRED)
find_package(bondcpp REQUIRED)

# catkin_package(DEPENDS diagnostic_msgs pluginlib roscpp rospy xmlrpcpp bondcpp
#    INCLUDE_DIRS include
#    LIBRARIES ${PROJECT_NAME})

#set(REQUIRED_diagnostic_msgs_VERSION_Indigo "1.11.9")
#set(REQUIRED_diagnostic_msgs_VERSION_Jade "1.12.4")
#if( ${diagnostic_msgs_VERSION} VERSION_LESS ${REQUIRED_diagnostic_msgs_VERSION_Indigo})
#  message(FATAL_ERROR "diagnostic_msgs version ${REQUIRED_diagnostic_msgs_VERSION_Indigo} or newer is required to build diagnotic_aggregator on ROS Indigo")
#endif()
#if( (${diagnostic_msgs_VERSION} VERSION_EQUAL "1.12.0") OR
#    (${diagnostic_msgs_VERSION} VERSION_GREATER "1.12.0" AND
#     ${diagnostic_msgs_VERSION} VERSION_LESS ${REQUIRED_diagnostic_msgs_VERSION_Jade}))
#  message(FATAL_ERROR "diagnostic_msgs version ${REQUIRED_diagnostic_msgs_VERSION_Jade} or newer is required to build diagnotic_aggregator on ROS Jade")
#endif()

find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${diagnostic_msgs_INCLUDE_DIRS} ${pluginlib_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${xmlrpcpp_INCLUDE_DIRS} ${boncpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} gtest-1.7.0/include)

add_library(${PROJECT_NAME}
  src/status_item.cpp
  src/analyzer_group.cpp
  src/generic_analyzer.cpp
  src/discard_analyzer.cpp
  src/ignore_analyzer.cpp
  src/aggregator.cpp)
target_link_libraries(diagnostic_aggregator ${Boost_LIBRARIES}
                                            ${diagnostic_msgs_LIBRARIES}
					    ${pluginlib}
					    ${rclcpp}
					    ${xmlrpcpp}
					    ${bondcpp}
)
#add_dependencies(${PROJECT_NAME} diagnostic_msgs_generate_messages_cpp)

# Aggregator node 
add_executable(aggregator_node src/aggregator_node.cpp)
target_link_libraries(aggregator_node ${catkin_LIBRARIES}
                                      ${PROJECT_NAME}
)

# Analyzer loader allows other users to test that Analyzers load
add_executable(analyzer_loader test/analyzer_loader.cpp
                               gtest-1.7.0/gtest-all.cc)
target_link_libraries(analyzer_loader diagnostic_aggregator)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/launch/test_agg.launch)
  add_rostest(test/launch/test_add_agg.launch)

  # Test Analyzer loader
  add_rostest(test/launch/test_loader.launch)
  add_rostest(test/launch/test_expected_stale.launch)
  add_rostest(test/launch/test_multiple_match.launch)
endif()

#catkin_install_python(
#        PROGRAMS scripts/add_analyzers
#        DESTINATION bin
#)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include
)
install(FILES analyzer_plugins.xml
        DESTINATION share
)
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib
)
install(TARGETS aggregator_node analyzer_loader
        DESTINATION bin
)

ament_package()
