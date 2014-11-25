cmake_minimum_required(VERSION 2.8.3)
project(diagnostic_common_diagnostics)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp diagnostic_updater)

catkin_python_setup()

catkin_package()

catkin_python_setup()


include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(libsensors_monitor src/libsensors_monitor.cpp src/libsensors_chip.cpp)
target_link_libraries(libsensors_monitor sensors ${catkin_LIBRARIES})


install(PROGRAMS
  src/diagnostic_common_diagnostics/hd_monitor.py
  src/diagnostic_common_diagnostics/ntp_monitor.py
  src/diagnostic_common_diagnostics/sensors_monitor.py
  src/diagnostic_common_diagnostics/tf_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS libsensors_monitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
