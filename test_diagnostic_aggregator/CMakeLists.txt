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
rosbuild_add_library(test_diagnostic_aggregator
  src/match_no_analyze_analyzer.cpp
  src/fail_init_analyzer.cpp)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_rostest(test/launch/test_match_no_analyze_analyzer_load.launch)

# Tests that an item can be matched, not analyzed, and reported in "Other"
rosbuild_add_rostest(test/launch/test_match_no_analyze.launch)

# Tests that item can be matched, not analyzed, but analyzed by another Analyzer
rosbuild_add_rostest(test/launch/test_match_then_analyze.launch)

# Tests that analyzers that fail to load will report an error in the diagnostics
rosbuild_add_rostest(test/launch/test_fail_init.launch)

# Tests that analyzers specified both with and without package name load correctly
rosbuild_add_rostest(test/launch/test_dual_analyzer_load.launch)