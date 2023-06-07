cmake_minimum_required(VERSION 3.5)
project(diagnostic_common_diagnostics)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    ${PROJECT_NAME}/ntp_monitor.py
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(
        test_ntp_monitor
        test/systemtest/test_ntp_monitor.py
        TIMEOUT 10)
endif()

ament_package()