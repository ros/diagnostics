cmake_minimum_required(VERSION 2.8.3)
project(diagnostic_analysis)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED diagnostic_msgs rosbag roslib rostest)

catkin_package(DEPENDS diagnostic_msgs rosbag roslib)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/bag_csv_test.py)
endif()

catkin_python_setup()
