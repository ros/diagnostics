cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

rosbuild_add_boost_directories()
rosbuild_add_library(diagnostic_aggregator
  src/generic_analyzer.cpp
  #src/other_analyzer.cpp
#  src/component.cpp
#  src/component_analyzer.cpp
  src/status_item.cpp
  src/aggregator.cpp)
rosbuild_link_boost(diagnostic_aggregator system)


#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(aggregator_node src/aggregator_node.cpp)
target_link_libraries(aggregator_node diagnostic_aggregator)

rosbuild_add_rostest(test/test_agg.launch)