cmake_minimum_required(VERSION 2.8.3)
project(diagnostic_common_diagnostics)

# Support C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Load catkin and all dependencies required for this package
#find_package(catkin REQUIRED COMPONENTS diagnostic_updater)
find_package(diagnostic_updater REQUIRED)

catkin_python_setup()

# catkin_package()

catkin_python_setup()

install(PROGRAMS
  src/diagnostic_common_diagnostics/hd_monitor.py
  src/diagnostic_common_diagnostics/ntp_monitor.py
  src/diagnostic_common_diagnostics/sensors_monitor.py
  src/diagnostic_common_diagnostics/tf_monitor.py
  DESTINATION bin)

ament_package()
