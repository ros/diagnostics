cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(cmdline_selftest)

rospack_add_executable(selftest selftest.cpp)

