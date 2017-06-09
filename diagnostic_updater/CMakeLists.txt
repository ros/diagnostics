cmake_minimum_required(VERSION 2.8.3)
project(diagnostic_updater)

# Support C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Load catkin and all dependencies required for this package
#find_package(catkin REQUIRED diagnostic_msgs roscpp std_msgs)
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_time REQUIRED)

# Include boost
find_package(Boost REQUIRED system)
link_directories(${Boost_LIBRARY_DIRS})

set(INCLUDE_DIRS
  include
  ${diagnostic_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${ros2_time_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(LIBS
  ${diagnostic_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${ros2_time_LIBRARIES}
  ${Boost_LIBRARIES}
)

#catkin_python_setup()

#catkin_package(DEPENDS diagnostic_msgs roscpp std_msgs
#               INCLUDE_DIRS include
#)

include_directories(${INCLUDE_DIRS})
add_executable(example src/example.cpp)
target_link_libraries(example ${LIBS})

if(CATKIN_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # TODO: need to support rostest
  #find_package(rostest REQUIRED)
  ament_add_gtest(diagnostic_updater_test test/diagnostic_updater_test.cpp)
  # add_rostest(test/diagnostic_updater_fast_test.xml)
  target_link_libraries(diagnostic_updater_test ${LIBS})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME} 
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(ros2_time)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
