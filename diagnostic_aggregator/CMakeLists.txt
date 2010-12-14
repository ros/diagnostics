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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_boost_directories()
rosbuild_add_library(diagnostic_aggregator
  src/status_item.cpp
  src/analyzer_group.cpp
  src/generic_analyzer.cpp
  src/discard_analyzer.cpp
  src/ignore_analyzer.cpp
  src/aggregator.cpp)
rosbuild_link_boost(diagnostic_aggregator system regex)

# Aggregator node 
rosbuild_add_executable(aggregator_node src/aggregator_node.cpp)
target_link_libraries(aggregator_node diagnostic_aggregator)

rosbuild_add_rostest(test/launch/test_agg.launch)

# Analyzer loader allows other users to test that Analyzers load
rosbuild_add_executable(analyzer_loader test/analyzer_loader.cpp)
target_link_libraries(analyzer_loader diagnostic_aggregator)
rosbuild_add_gtest_build_flags(analyzer_loader)
set_target_properties(analyzer_loader PROPERTIES EXCLUDE_FROM_ALL FALSE)


# Test Analyzer loader
rosbuild_add_rostest(test/launch/test_loader.launch)
rosbuild_add_rostest(test/launch/test_expected_stale.launch)
rosbuild_add_rostest(test/launch/test_multiple_match.launch)
