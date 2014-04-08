cmake_minimum_required(VERSION 2.8.3)
project(self_test)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED diagnostic_msgs diagnostic_updater roscpp rostest)

catkin_package(DEPENDS diagnostic_msgs diagnostic_updater roscpp
               INCLUDE_DIRS include
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})

catkin_add_gtest(selftest_rostest src/selftest_rostest.cpp)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(selftest_rostest ${Boost_LIBRARIES}
                                       ${catkin_LIBRARIES}
)
# Following line needed because of #ros3178.
set_target_properties(selftest_rostest PROPERTIES EXCLUDE_FROM_ALL FALSE)

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

add_subdirectory(test EXCLUDE_FROM_ALL)

add_rostest(test/launch/nominal_test.launch)
add_rostest(test/launch/no_id_test.launch)
add_rostest(test/launch/error_test.launch)
add_rostest(test/launch/exception_test.launch)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS run_selftest selftest_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
