cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(self_test)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

gensrv()

rospack_add_boost_directories()

rospack_add_executable(selftest_rostest src/selftest_rostest.cpp)
rospack_link_boost(selftest_rostest thread)
rospack_add_gtest_build_flags(selftest_rostest)

rospack_add_executable(run_selftest src/run_selftest.cpp)
rospack_link_boost(run_selftest thread)

rospack_add_executable(selftest_example src/selftest_example.cpp)
rospack_link_boost(selftest_example thread)
