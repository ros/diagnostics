cmake_minimum_required(VERSION 3.0.2)
project(diagnostic_common_diagnostics)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS diagnostic_updater)

catkin_python_setup()

catkin_package()

catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Test CPU Monitor
  add_rostest(test/launch/test_cpu_monitor.launch)
  add_rostest(test/launch/test_cpu_monitor_level_ok.launch)
  add_rostest(test/launch/test_cpu_monitor_level_warn.launch)
endif()

catkin_install_python(PROGRAMS
  src/diagnostic_common_diagnostics/cpu_monitor.py
  src/diagnostic_common_diagnostics/hd_monitor.py
  src/diagnostic_common_diagnostics/ntp_monitor.py
  src/diagnostic_common_diagnostics/sensors_monitor.py
  src/diagnostic_common_diagnostics/tf_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
