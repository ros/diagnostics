cmake_minimum_required(VERSION 2.8.3)
project(self_test)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED diagnostic_msgs diagnostic_updater roscpp rostest)

catkin_package(DEPENDS diagnostic_msgs diagnostic_updater roscpp
               INCLUDE_DIRS include
)

include_directories(include gtest-1.7.0/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(run_selftest src/run_selftest.cpp)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(run_selftest ${Boost_LIBRARIES}
                                   ${catkin_LIBRARIES}
)

add_executable(selftest_example src/selftest_example.cpp)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(selftest_example ${Boost_LIBRARIES}
                                       ${catkin_LIBRARIES}
)

add_executable(selftest_rostest src/selftest_rostest.cpp
                                gtest-1.7.0/gtest-all.cc)
target_link_libraries(selftest_rostest ${Boost_LIBRARIES}
                                       ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)

  add_rostest(test/launch/nominal_test.launch)
  add_rostest(test/launch/no_id_test.launch)
  add_rostest(test/launch/error_test.launch)
  add_rostest(test/launch/exception_test.launch)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS run_selftest selftest_example selftest_rostest
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
