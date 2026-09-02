cmake_minimum_required(VERSION 3.20)

# 根 CMake 只负责组装与原车 /home/5G/5G 同层级的子工程。
# main/vision 和 main/motion_control 也都可以单独编译，便于树莓派上分步部署。
project(xt_netrc_workspace VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_subdirectory(main/vision)
add_subdirectory(main/motion_control)
add_subdirectory(main/ground_projection)
add_subdirectory(main/camera_gimbal)
add_subdirectory(main/visual_distance_test)
