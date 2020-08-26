cmake_minimum_required(VERSION 3.0.2)
project(diagnostic_updater)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED diagnostic_msgs roscpp std_msgs)

catkin_python_setup()

catkin_package(DEPENDS diagnostic_msgs roscpp std_msgs
               INCLUDE_DIRS include
	       LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(example src/example.cpp)
add_library(${PROJECT_NAME}
  src/timestamp_status.cpp)
target_link_libraries(example ${catkin_LIBRARIES})

#set(LOCAL_GTEST_DIR "gtest-1.7.0")
# gtest could be included in ${catkin_INCLUDE_DIRS}, prepend local gtest include directory so it's searched first
#include_directories(BEFORE ${LOCAL_GTEST_DIR}/include)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(diagnostic_updater_test test/diagnostic_updater_test.xml test/diagnostic_updater_test.cpp)
  add_rostest(test/diagnostic_updater_fast_test.xml)
  target_link_libraries(diagnostic_updater_test ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
