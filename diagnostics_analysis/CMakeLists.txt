cmake_minimum_required(VERSION 2.4.6)
# Unit test fails when build type changes to RelWithDebInfo (-O2)
set(ROS_BUILD_TYPE RelWithDebInfo)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(diagnostics_analysis)

rospack_add_pyunit(test/bag_csv_test.py)