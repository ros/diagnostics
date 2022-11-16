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

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    file(GLOB_RECURSE AMENT_LINT_AUTO_FILE_EXCLUDE
        # Exclude all the ROS1 files in src directory
        src/*.py
        # Exclude all the ROS1 files in test directory
        test/*.py
    )
    message(AUTHOR_WARNING
        "Ament lint auto tests are disabled on the following: "
        ${AMENT_LINT_AUTO_FILE_EXCLUDE}
    )
    ament_lint_auto_find_test_dependencies()
endif()
ament_package()