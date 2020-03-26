cmake_minimum_required(VERSION 3.0.2)
project(diagnostic_analysis)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED diagnostic_msgs rosbag roslib)

catkin_package(DEPENDS diagnostic_msgs rosbag roslib)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/bag_csv_test.py)
endif()

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/export_csv.py scripts/sparse_csv.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
