cmake_minimum_required(VERSION 3.16)

project(xtnetrc_gps_speed_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(xtnetrc_gps_speed STATIC
    src/wit_sensor.cpp
    src/speed_loop.cpp
)
target_include_directories(xtnetrc_gps_speed PUBLIC include)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(xtnetrc_gps_speed PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Only the serial adapter object is linked; monitor contains no actuator calls.
if(NOT TARGET xtnetrc_hardware)
    add_subdirectory(../hardware hardware_build)
endif()
add_executable(xtnetrc_gps_speed_monitor src/monitor.cpp)
target_link_libraries(xtnetrc_gps_speed_monitor PRIVATE xtnetrc_gps_speed xtnetrc_hardware)

include(CTest)
if(BUILD_TESTING)
    add_executable(xtnetrc_gps_speed_test tests/gps_speed_test.cpp)
    target_link_libraries(xtnetrc_gps_speed_test PRIVATE xtnetrc_gps_speed)
    add_test(NAME xtnetrc_gps_speed_test COMMAND xtnetrc_gps_speed_test)
endif()
