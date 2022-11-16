cmake_minimum_required(VERSION 3.5)
project(diagnostic_common_diagnostics)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    ${PROJECT_NAME}/ntp_monitor.py
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()