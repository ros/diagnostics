find_package(ament_cmake_gtest REQUIRED)

macro(add_self_test target_name file_path)
  ament_add_gtest(${target_name} ${file_path})
  if(TARGET ${target_name})
    ament_target_dependencies(
      ${target_name}
      "diagnostic_msgs"
      "diagnostic_updater"
      "rclcpp"
    )
    target_include_directories(${target_name}
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
      $<INSTALL_INTERFACE:include>
    )
  endif()
endmacro()

add_self_test(test_nominal_selftest nominal_selftest.cpp)
add_self_test(test_no_id_selftest no_id_selftest.cpp)
add_self_test(test_exception_selftest exception_selftest.cpp)
add_self_test(test_error_selftest error_selftest.cpp)
