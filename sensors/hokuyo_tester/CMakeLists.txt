cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(hokuyo_tester)

set(wxWidgets_USE_LIBS base core gl)

find_package(wxWidgets REQUIRED)

include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/src)

rospack_add_library(hokuyotester   src/hokuyo_tester/hokuyo_tester.cpp
                                   src/hokuyo_tester/gen_hokuyo_tester.cpp)

rospack_add_executable(gui         src/gui/gui.cpp)
rospack_add_executable(test_hokuyo src/standalone/standalone.cpp)

target_link_libraries(gui hokuyotester ${wxWidgets_LIBRARIES})

