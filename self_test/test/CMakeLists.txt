catkin_add_gtest(nominal_selftest nominal_selftest.cpp)
target_link_libraries(nominal_selftest ${catkin_LIBRARIES})

catkin_add_gtest(exception_selftest exception_selftest.cpp)
target_link_libraries(exception_selftest ${catkin_LIBRARIES})

catkin_add_gtest(error_selftest error_selftest.cpp)
target_link_libraries(error_selftest ${catkin_LIBRARIES})

catkin_add_gtest(no_id_selftest no_id_selftest.cpp)
target_link_libraries(no_id_selftest ${catkin_LIBRARIES})
