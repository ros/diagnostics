cmake_minimum_required(VERSION 3.5)
project(diagnostic_common_diagnostics)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    ${PROJECT_NAME}/cpu_monitor.py
    ${PROJECT_NAME}/ntp_monitor.py
    ${PROJECT_NAME}/ram_monitor.py
    ${PROJECT_NAME}/sensors_monitor.py
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(
        test_cpu_monitor
        test/systemtest/test_cpu_monitor.py
        TIMEOUT 10)
    add_launch_test(
        test/systemtest/test_ntp_monitor_launchtest.py
        TARGET ntp_monitor_launchtest
        TIMEOUT 20)
endif()

ament_package()
