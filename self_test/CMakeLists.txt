cmake_minimum_required(VERSION 3.5)
project(self_test)

# Support C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# Load catkin and all dependencies required for this package
#find_package(catkin REQUIRED diagnostic_msgs diagnostic_updater roscpp rostest)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
#link_directories(${Boost_LIBRARY_DIRS})
#include_directories(${Boost_INCLUDE_DIRS})

#catkin_package(DEPENDS diagnostic_msgs diagnostic_updater roscpp
#               INCLUDE_DIRS include
#)
set(INCLUDE_DIRS
  include
  ${diagnostic_msgs_INCLUDE_DIRS}
  ${diagnostic_updater_INCLUDE_DIRS}
  ${builtin_interfaces_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclpy_INCLUDE_DIRS}
)

set(LIBS
  ${diagnostic_msgs_LIBRARIES}
  ${diagnostic_updater_LIBRARIES}
  ${builtin_interfaces_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rcutils_LIBRARIES}
  ${rclpy_LIBRARIES}
)

include_directories(${INCLUDE_DIRS})

#include_directories(include gtest-1.7.0/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(run_selftest src/run_selftest.cpp)
target_link_libraries(run_selftest ${LIBS})

add_executable(selftest_example src/selftest_example.cpp)
target_link_libraries(selftest_example ${LIBS})

install(
  TARGETS
  run_selftest
  selftest_example
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_executable(nominal_selftest test/nominal_selftest.cpp)
  target_link_libraries(nominal_selftest ${LIBS})

  add_executable(exception_selftest test/exception_selftest.cpp)
  target_link_libraries(exception_selftest ${LIBS})

  add_executable(error_selftest test/error_selftest.cpp)
  target_link_libraries(error_selftest ${LIBS})

  add_executable(no_id_selftest test/no_id_selftest.cpp)
  target_link_libraries(no_id_selftest ${LIBS})

  install(
    TARGETS
    no_id_selftest
    nominal_selftest
    exception_selftest
    error_selftest
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(rclcpp)
ament_export_include_directories(${INCLUDE_DIRS})
ament_package()
