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
    set(_python_test_files
        test/test_ntp_monitor.py
        # Add other tests here like test_cpu_monitor.py once it's ported to ROS2
    )
    foreach(_test_path ${_python_test_files})
        get_filename_component(_test_name ${_test_path} NAME_WE)
        ament_add_pytest_test(${_test_name} ${_test_path}
            APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
            TIMEOUT 30
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        )
    endforeach()
endif()
ament_package()