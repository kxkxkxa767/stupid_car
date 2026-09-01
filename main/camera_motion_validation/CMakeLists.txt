cmake_minimum_required(VERSION 3.20)
project(xtnetrc_camera_motion_validation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc videoio)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
add_subdirectory(../motion_control motion_control_build)

add_executable(xtnetrc_camera_motion_validation
    src/main.cpp
    ../vision/src/lane_detector.cpp
)
target_include_directories(xtnetrc_camera_motion_validation PRIVATE
    ../vision/include
)
target_link_libraries(xtnetrc_camera_motion_validation PRIVATE
    xtnetrc_motion
    ${OpenCV_LIBS}
)
target_compile_options(xtnetrc_camera_motion_validation PRIVATE
    $<$<CXX_COMPILER_ID:AppleClang,Clang,GNU>:-Wall;-Wextra;-Wpedantic>
)
