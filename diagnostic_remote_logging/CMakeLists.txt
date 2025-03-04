cmake_minimum_required(VERSION 3.8)

project(diagnostic_remote_logging)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
    ament_cmake
    rclcpp
    rclcpp_components
    diagnostic_msgs
    CURL
)

foreach(dep ${dependencies})
    find_package(${dep} REQUIRED)
endforeach(dep)

include_directories(
    src/
    include/
)
add_library(influx_component SHARED src/influxdb.cpp)

ament_target_dependencies(influx_component ${dependencies})

ament_export_dependencies(influx_component ${dependencies})

target_compile_features(influx_component PUBLIC c_std_99 cxx_std_17)

rclcpp_components_register_node(
    influx_component
    PLUGIN "InfluxDB"
    EXECUTABLE influx
)
ament_export_targets(export_influx_component)
install(TARGETS influx_component
        EXPORT export_influx_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(unit_tests test/influx_line_protocol.cpp)
    target_include_directories(unit_tests PRIVATE include)
    target_link_libraries(unit_tests influx_component)
    ament_target_dependencies(unit_tests ${dependencies} ament_cmake_gtest)
endif()
ament_package()
