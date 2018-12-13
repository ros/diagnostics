cmake_minimum_required(VERSION 3.5)
project(bondcpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(Boost REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_search_module(UUID REQUIRED uuid)

find_package(rcutils REQUIRED)
find_package(smclibcpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bond REQUIRED)

set(INCLUDE_DIRS
  include
  ${smclibcpp_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${bond_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
  ${rcl_INCLUDE_DIRS})

set(LIBS
  ${rclcpp_LIBRARIES}
  ${smclibcpp_LIBRARIES}
  ${bond_LIBRARIES}
  ${rcl_LIBRARIES}
  ${rcutils_LIBRARIES})

include_directories(${INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/bond.cpp
  src/BondSM_sm.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp)
target_link_libraries(${PROJECT_NAME} ${LIBS} ${UUID_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()

