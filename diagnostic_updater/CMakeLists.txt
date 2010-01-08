cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_boost_directories()

rosbuild_add_executable(example src/example.cpp)

rosbuild_add_executable(diagnostic_updater_test test/diagnostic_updater_test.cpp)
rosbuild_add_gtest_build_flags(diagnostic_updater_test)
rosbuild_add_rostest(test/diagnostic_updater_test.xml)

