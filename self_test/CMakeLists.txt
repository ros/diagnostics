cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_boost_directories()

rosbuild_add_executable(selftest_rostest src/selftest_rostest.cpp)
rosbuild_link_boost(selftest_rostest thread)
rosbuild_add_gtest_build_flags(selftest_rostest)
# Following line needed because of #ros3178.
set_target_properties(selftest_rostest PROPERTIES EXCLUDE_FROM_ALL FALSE)

rosbuild_add_executable(run_selftest src/run_selftest.cpp)
rosbuild_link_boost(run_selftest thread)

rosbuild_add_executable(selftest_example src/selftest_example.cpp)
rosbuild_link_boost(selftest_example thread)

add_subdirectory(test EXCLUDE_FROM_ALL)

rosbuild_add_rostest(test/launch/nominal_test.launch)
rosbuild_add_rostest(test/launch/no_id_test.launch)
rosbuild_add_rostest(test/launch/error_test.launch)
rosbuild_add_rostest(test/launch/exception_test.launch)
