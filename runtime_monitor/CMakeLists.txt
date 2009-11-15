cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

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
  rosbuild_add_rostest(test/runtime_mon_test.launch)
endif(disp)