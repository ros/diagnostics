cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(self_test)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(run_selftest src/run_selftest.cpp)
rospack_add_executable(selftest_example src/selftest_example.cpp)

