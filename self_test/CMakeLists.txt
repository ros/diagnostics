cmake_minimum_required(VERSION 2.8.3)
project(self_test)

# Support C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Load catkin and all dependencies required for this package
#find_package(catkin REQUIRED diagnostic_msgs diagnostic_updater roscpp rostest)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rostest REQUIRED)

# catkin_package(DEPENDS diagnostic_msgs diagnostic_updater roscpp
#               INCLUDE_DIRS include
#)

include_directories(include gtest-1.7.0/include ${diagnostic_msgs_INCLUDE_DIRS} ${diagnostic_updater_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${rostest_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(run_selftest src/run_selftest.cpp)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(run_selftest ${Boost_LIBRARIES}
                                   ${diagnostic_msgs_LIBRARIES}
                                   ${diagnostic_updater_LIBRARIES}
				   ${rclcpp_LIBRARIES}
			           ${rostest_LIBRARIES}
)

add_executable(selftest_example src/selftest_example.cpp)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(selftest_example ${Boost_LIBRARIES}
                                   ${diagnostic_msgs_LIBRARIES}
                                   ${diagnostic_updater_LIBRARIES}
                                   ${rclcpp_LIBRARIES}
                                   ${rostest_LIBRARIES}
)

add_executable(selftest_rostest src/selftest_rostest.cpp
                                gtest-1.7.0/gtest-all.cc)
target_link_libraries(selftest_rostest ${Boost_LIBRARIES}
                                   ${diagnostic_msgs_LIBRARIES}
                                   ${diagnostic_updater_LIBRARIES}
                                   ${rclcpp_LIBRARIES}
                                   ${rostest_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)

  add_rostest(test/launch/nominal_test.launch)
  add_rostest(test/launch/no_id_test.launch)
  add_rostest(test/launch/error_test.launch)
  add_rostest(test/launch/exception_test.launch)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include
)

install(TARGETS run_selftest selftest_example selftest_rostest
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
