cmake_minimum_required(VERSION 3.5)
project(test_bond)

# Support C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(test_bond_srv_gen REQUIRED)
find_package(bondcpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_search_module(UUID REQUIRED uuid)

set(INCLUDE_DIRS
  ${rclcpp_INCLUDE_DIRS}
  ${test_bond_srv_gen_INCLUDE_DIRS}
  ${bondcpp_INCLUDE_DIRS}
)
set(LIBS
  ${rclcpp_LIBRARIES}
  ${bondcpp_LIBRARIES}
  ${test_bond_srv_gen_LIBRARIES}
  ${UUID_LIBRARIES}
)
include_directories(${INCLUDE_DIRS})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_callbacks_cpp test/test_callbacks_cpp.cpp)
  if(TARGET test_callbacks_cpp)
    target_link_libraries(test_callbacks_cpp ${LIBS} ${GTEST_LIBRARIES})
  endif()
  add_executable(test_callbacks_cpp_exc test/test_callbacks_cpp_exc.cpp)
  target_link_libraries(test_callbacks_cpp_exc ${LIBS})
  add_executable(exercise_bond_cpp_exc test/exercise_bond_cpp_exc.cpp)
  target_link_libraries(exercise_bond_cpp_exc ${LIBS})
  install(
          TARGETS
          test_callbacks_cpp_exc
          exercise_bond_cpp_exc
          DESTINATION lib/${PROJECT_NAME})

endif()
ament_package()
