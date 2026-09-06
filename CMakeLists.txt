cmake_minimum_required(VERSION 3.20)

# 根 CMake 只负责组装与原车 /home/5G/5G 同层级的子工程。
# main/vision 和 main/motion_control 也都可以单独编译，便于树莓派上分步部署。
project(xt_netrc_workspace VERSION 0.4.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

option(XTNETRC_BUILD_VISION "Build OpenCV perception, replay and live runtime" ON)
option(XTNETRC_BUILD_GUI "Build desktop camera GUI" ON)
option(XTNETRC_WITH_PIGPIO "Build Raspberry Pi motor adapter; runtime stays opt-in" OFF)
add_subdirectory(main/hardware)
add_subdirectory(main/motion_control)
add_subdirectory(main/gps_speed_control)
add_subdirectory(main/camera_gimbal)
if(XTNETRC_BUILD_VISION)
    add_subdirectory(main/vision)
    add_subdirectory(main/ground_projection)
    add_subdirectory(main/camera_io)
    add_subdirectory(main/visual_distance_test)
    add_subdirectory(main/camera_motion_validation)
endif()
add_subdirectory(main/car_runtime)
