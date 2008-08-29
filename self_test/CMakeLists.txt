cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(self_test)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(selftest src/selftest.cpp)
rospack_add_executable(selftest_server src/selftest_server.cpp)

