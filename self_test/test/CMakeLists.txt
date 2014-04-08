add_executable(nominal_selftest nominal_selftest.cpp)
target_link_libraries(nominal_selftest ${catkin_LIBRARIES})

add_executable(exception_selftest exception_selftest.cpp)
target_link_libraries(exception_selftest ${catkin_LIBRARIES})

add_executable(error_selftest error_selftest.cpp)
target_link_libraries(error_selftest ${catkin_LIBRARIES})

add_executable(no_id_selftest no_id_selftest.cpp)
target_link_libraries(no_id_selftest ${catkin_LIBRARIES})
