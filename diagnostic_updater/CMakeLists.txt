cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(diagnostic_updater)

rospack_add_boost_directories()

rospack_add_executable(test/diagnostic_updater_test test/diagnostic_updater_test.cpp)
rospack_add_gtest_build_flags(test/diagnostic_updater_test)
rospack_add_rostest(test/diagnostic_updater_test.xml)

