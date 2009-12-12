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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# Remove in ROS 0.10
macro(rosbuild_check_for_display var)
 execute_process(COMMAND "xdpyinfo"
                 OUTPUT_VARIABLE _dummy
                 ERROR_VARIABLE _dummy
                 RESULT_VARIABLE _xdpyinfo_failed)
 if(_xdpyinfo_failed)
   set(${var} 0)
 else(_xdpyinfo_failed)
   set(${var} 1)
 endif(_xdpyinfo_failed)
endmacro(rosbuild_check_for_display)

rosbuild_check_for_display(disp)
if(disp)
  rosbuild_add_rostest(test/robot_mon_test.launch)
endif(disp)