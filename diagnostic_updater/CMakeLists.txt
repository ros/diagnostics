cmake_minimum_required(VERSION 3.5)
project(diagnostic_updater)

# Support C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)

set(INCLUDE_DIRS
  include
  ${diagnostic_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

set(LIBS
  ${diagnostic_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

include_directories(${INCLUDE_DIRS})
add_executable(example src/example.cpp)
target_link_libraries(example ${LIBS})

############################################################
# Define tests
#

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(diagnostic_updater_test test/diagnostic_updater_test.cpp)
  target_link_libraries(diagnostic_updater_test ${LIBS})

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(diagnostic_updater_test.py "test/diagnostic_updater_test.py")
  ament_add_pytest_test(test_DiagnosticStatusWrapper.py "test/test_DiagnosticStatusWrapper.py")
endif()
############################################################
# Define installation rules
#
ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# Export include dependencies, directories, and libraries
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rclpy)
ament_export_include_directories(${INCLUDE_DIRS})

# Must be at the bottom
ament_package()
