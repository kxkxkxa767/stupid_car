cmake_minimum_required(VERSION 3.20)
project(xtnetrc_camera_motion_validation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc videoio)
if(NOT TARGET xtnetrc_motion)
    add_subdirectory(../motion_control motion_control_build)
endif()
if(NOT TARGET xtnetrc_lane_detector)
    set(XTNETRC_BUILD_GUI OFF)
    add_subdirectory(../vision vision_build)
endif()

add_executable(xtnetrc_camera_motion_validation
    src/main.cpp
)
target_include_directories(xtnetrc_camera_motion_validation PRIVATE
    ../vision/include
)
target_link_libraries(xtnetrc_camera_motion_validation PRIVATE
    xtnetrc_motion
    xtnetrc_lane_detector
    ${OpenCV_LIBS}
)
target_compile_options(xtnetrc_camera_motion_validation PRIVATE
    $<$<CXX_COMPILER_ID:AppleClang,Clang,GNU>:-Wall;-Wextra;-Wpedantic>
)
