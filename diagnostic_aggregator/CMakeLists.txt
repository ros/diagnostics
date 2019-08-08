cmake_minimum_required(VERSION 3.5)
project(diagnostic_aggregator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bondcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/status_item.cpp
  src/analyzer_group.cpp
  src/aggregator.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  "bondcpp"
  "diagnostic_msgs"
  "pluginlib"
  "rclcpp"
  "std_msgs"
)

set(PLUGIN_LIB "${PROJECT_NAME}_analyzers")
add_library(${PLUGIN_LIB} SHARED
  src/generic_analyzer.cpp
  src/discard_analyzer.cpp
  src/ignore_analyzer.cpp)
target_include_directories(${PLUGIN_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PLUGIN_LIB}
  "bondcpp"
  "diagnostic_msgs"
  "pluginlib"
  "rclcpp"
  "std_msgs"
)
target_link_libraries(${PLUGIN_LIB}
  ${PROJECT_NAME})
target_compile_definitions(${PLUGIN_LIB}
  PRIVATE "DIAGNOSTICS_DEFAULT_ANALYZER_PLUGINS_BUILDING_LIBRARY")

# Aggregator node
add_executable(aggregator_node src/aggregator_node.cpp)
target_link_libraries(aggregator_node
  ${PROJECT_NAME})
install(
  TARGETS aggregator_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(analyzer_loader test/analyzer_loader.cpp)
  if(TARGET analyzer_loader)
    target_include_directories(analyzer_loader PUBLIC
      ${rclcpp_INCLUDE_DIRS}
    )
    target_link_libraries(analyzer_loader
      ${PROJECT_NAME}
      ${PLUGIN_LIB})
  endif()

  # TODO(Karsten1987): Re-enable when bondpy is ported
  # find_package(ament_cmake_pytest REQUIRED)
  # ament_add_pytest_test(diagnostic_aggregator_test.py "test/add_analyzers_test.py")
endif()

ament_python_install_package(${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME} ${PLUGIN_LIB}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install Demo
set(ANALYZER_PARAMS_FILEPATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/demo_pub.yaml")
configure_file(demo/demo.launch.py.in demo.launch.py @ONLY)
install( # launch descriptor
  FILES ${CMAKE_CURRENT_BINARY_DIR}/demo.launch.py
  DESTINATION share/${PROJECT_NAME}
)
install( # example publisher
  FILES demo/demo_pub.py
  DESTINATION lib/${PROJECT_NAME}
)
install( # example aggregator configration
  FILES demo/demo_pub.yaml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(${PLUGIN_LIB} plugin_description.xml)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(bondcpp)
ament_export_dependencies(bondpy)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)

ament_package()
