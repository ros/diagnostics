cmake_minimum_required(VERSION 2.4.6)

set(ROS_BUILD_TYPE RelWithDebInfo)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rospack_add_pyunit(test/bag_csv_test.py)