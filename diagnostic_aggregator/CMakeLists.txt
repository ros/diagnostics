cmake_minimum_required(VERSION 3.20)
project(diagnostic_aggregator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/status_item.cpp
  src/analyzer_group.cpp
  src/aggregator.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${diagnostic_msgs_TARGETS}
  ${std_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "DIAGNOSTIC_AGGREGATOR_BUILDING_DLL")

# see https://github.com/pybind/pybind11/commit/ba33b2fc798418c8c9dfe801c5b9023d3703f417
if(NOT WIN32)
  target_compile_options(${PROJECT_NAME} PRIVATE -Wdeprecated)
endif()

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

set(ANALYZERS "${PROJECT_NAME}_analyzers")
add_library(${ANALYZERS} SHARED
  src/generic_analyzer.cpp
  src/discard_analyzer.cpp
  src/ignore_analyzer.cpp)
target_include_directories(${ANALYZERS} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${ANALYZERS} PUBLIC
  ${PROJECT_NAME}
  ${diagnostic_msgs_TARGETS}
  ${std_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
)
target_compile_definitions(${ANALYZERS}
  PRIVATE "DIAGNOSTIC_AGGREGATOR_BUILDING_DLL")

# prevent pluginlib from using boost
target_compile_definitions(${ANALYZERS} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Aggregator node
add_executable(aggregator_node src/aggregator_node.cpp)
target_link_libraries(aggregator_node
  ${PROJECT_NAME})

# Add analyzer
add_executable(add_analyzer src/add_analyzer.cpp)
target_link_libraries(add_analyzer PUBLIC ${rcl_interfaces_TARGETS} rclcpp::rclcpp)

# Testing macro
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  file(TO_CMAKE_PATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/aggregator_node" AGGREGATOR_NODE)
  file(TO_CMAKE_PATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/add_analyzer" ADD_ANALYZER)
  file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/test_listener.py" TEST_LISTENER)

  set(create_analyzers_tests
    "primitive_analyzers"
    "all_analyzers"
    "analyzer_group"
    "empty_root_path")

  foreach(test_name ${create_analyzers_tests})
    file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/${test_name}.yaml" PARAMETER_FILE)
    file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/expected_output/create_${test_name}" EXPECTED_OUTPUT)

    configure_file(
      "test/test_create_analyzers.launch.py.in"
      "test_create_${test_name}.launch.py"
      @ONLY
    )
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test_create_${test_name}.launch.py"
      TARGET "test_create_${test_name}"
      TIMEOUT 30
      ENV
    )
  endforeach()

  set(analyzers_output_tests
    "primitive_analyzers"
    "all_analyzers"
    "analyzer_group"
    "empty_root_path")

  foreach(test_name ${analyzers_output_tests})
    file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/${test_name}.yaml" PARAMETER_FILE)
    file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/expected_output/output_${test_name}" EXPECTED_OUTPUT)

    configure_file(
      "test/test_analyzers_output.launch.py.in"
      "test_output_${test_name}.launch.py"
      @ONLY
    )
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test_output_${test_name}.launch.py"
      TARGET "test_output_${test_name}"
      TIMEOUT 30
      ENV
    )
  endforeach()

  set(add_analyzers_tests
    "primitive_analyzers"
    "all_analyzers"
    "analyzer_group"
    "empty_root_path")

  foreach(test_name ${add_analyzers_tests})
    file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/default.yaml" PARAMETER_FILE)
    file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/${test_name}.yaml" ADD_PARAMETER_FILE)
    file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/expected_output/add_${test_name}" EXPECTED_OUTPUT)

    configure_file(
      "test/test_add_analyzers.launch.py.in"
      "test_add_${test_name}.launch.py"
      @ONLY
    )
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test_add_${test_name}.launch.py"
      TARGET "test_add_${test_name}"
      TIMEOUT 30
      ENV
    )
  endforeach()

  add_launch_test(
    test/test_critical_pub.py
    TIMEOUT 30
  )

  # Each entry: "case_name|FOO_DISCARD|FOO_STATUS|BAR_DISCARD|BAR_STATUS|AGG_EXPECTED"
  # Statuses use DiagnosticStatus integer values: OK=0, WARN=1, ERROR=2, STALE=3
  # A status of "None" means the diagnostic is never published (missing).
  set(discard_behavior_tests
    # CASE 1: both 'foo' and 'bar' are marked discard_stale := true
    #  --> the aggregator should roll up to OK=0, because the stale statuses are discarded.
    "case_1_01|True|None|True|None|0"
    "case_1_02|True|3|True|3|0"
    "case_1_03|True|None|True|3|0"
    "case_1_04|True|3|True|None|0"
    # CASE 2: both 'foo' and 'bar' are marked discard_stale := false
    #  --> the aggregator should roll up to STALE=3, as long as they aggregated are either
    # missing or stale, because the stale statuses are *not* discarded.
    "case_2_01|False|None|False|None|3"
    "case_2_02|False|3|False|3|3"
    "case_2_03|False|None|False|3|3"
    "case_2_04|False|3|False|None|3"
    # CASE 3: one of 'foo' or 'bar' are marked discard_stale := true and the corresponding status
    # is missing or stale while the other is OK.
    #  --> the aggregator should roll up to OK, because the stale status is discarded.
    "case_3_01|True|None|False|0|0"
    "case_3_02|True|3|False|0|0"
    "case_3_03|True|0|False|0|0"
    "case_3_04|False|0|True|None|0"
    "case_3_05|False|0|True|3|0"
    "case_3_06|False|0|True|0|0"
    # CASE 4: one of 'foo' or 'bar' are marked discard_stale := true and the corresponding status
    # is missing or stale while the other is also missing or stale.
    #  --> the aggregator should roll up to STALE=3
    "case_4_01|True|3|False|None|3"
    "case_4_02|True|None|False|None|3"
    "case_4_03|True|None|False|3|3"
    "case_4_04|True|3|False|3|3"
    "case_4_05|False|3|True|None|3"
    "case_4_06|False|None|True|None|3"
    "case_4_07|False|None|True|3|3"
    "case_4_08|False|3|True|3|3"
    # CASE 5: one of 'foo' or 'bar' are marked discard_stale := true and the corresponding status
    # is OK while the other is missing or stale.
    #  --> the aggregator should roll up to STALE, because it has a higher severity than OK.
    "case_5_01|True|0|False|3|3"
    "case_5_02|True|0|False|None|3"
    "case_5_03|False|3|True|0|3"
    "case_5_04|False|None|True|0|3"
  )

  foreach(test_record IN LISTS discard_behavior_tests)
    string(REPLACE "|" ";" test_parts "${test_record}")
    list(GET test_parts 0 TEST_CASE_NAME)
    list(GET test_parts 1 FOO_DISCARD)
    list(GET test_parts 2 FOO_STATUS)
    list(GET test_parts 3 BAR_DISCARD)
    list(GET test_parts 4 BAR_STATUS)
    list(GET test_parts 5 AGG_EXPECTED)

    configure_file(
      "test/test_discard_behavior.launch.py.in"
      "test_discard_behavior_${TEST_CASE_NAME}.launch.py"
      @ONLY
    )
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test_discard_behavior_${TEST_CASE_NAME}.launch.py"
      TARGET "test_discard_behavior_${TEST_CASE_NAME}"
      TIMEOUT 30
      ENV
    )
  endforeach()

  ament_add_pytest_test(test_publish_values
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_publish_values.py"
    TIMEOUT 60
  )
endif()

install(
  TARGETS aggregator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS add_analyzer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} ${ANALYZERS}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

# Install Example
set(ANALYZER_PARAMS_FILEPATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/example_analyzers.yaml")
set(ADD_ANALYZER_PARAMS_FILEPATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/example_add_analyzers.yaml")
configure_file(example/example.launch.py.in example.launch.py @ONLY)
install( # launch descriptor
  FILES ${CMAKE_CURRENT_BINARY_DIR}/example.launch.py
  DESTINATION share/${PROJECT_NAME}
)
install( # example publisher
  PROGRAMS example/example_pub.py
  DESTINATION lib/${PROJECT_NAME}
)
install( # example aggregator configration
  FILES example/example_analyzers.yaml example/example_add_analyzers.yaml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugin_description.xml)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(pluginlib)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)

ament_package()
