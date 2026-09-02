cmake_minimum_required(VERSION 3.20)
project(xtnetrc_visual_distance_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc video videoio)

add_library(xtnetrc_visual_odometry STATIC
    src/visual_odometry.cpp
    ../ground_projection/src/ground_projector.cpp
)
target_include_directories(xtnetrc_visual_odometry PUBLIC
    include
    ../ground_projection/include
)
target_link_libraries(xtnetrc_visual_odometry PUBLIC ${OpenCV_LIBS})
target_compile_options(xtnetrc_visual_odometry PRIVATE
    $<$<CXX_COMPILER_ID:AppleClang,Clang,GNU>:-Wall;-Wextra;-Wpedantic>
)

add_executable(xtnetrc_visual_distance_test
    src/main.cpp
    src/pigpio_vehicle.cpp
)
target_include_directories(xtnetrc_visual_distance_test PRIVATE include)
target_link_libraries(xtnetrc_visual_distance_test PRIVATE
    xtnetrc_visual_odometry
    ${CMAKE_DL_LIBS}
)
target_compile_options(xtnetrc_visual_distance_test PRIVATE
    $<$<CXX_COMPILER_ID:AppleClang,Clang,GNU>:-Wall;-Wextra;-Wpedantic>
)
set_target_properties(xtnetrc_visual_distance_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

include(CTest)
if(BUILD_TESTING)
    add_executable(xtnetrc_visual_odometry_test tests/visual_odometry_test.cpp)
    target_link_libraries(xtnetrc_visual_odometry_test PRIVATE
        xtnetrc_visual_odometry
    )
    target_compile_definitions(xtnetrc_visual_odometry_test PRIVATE
        XTNETRC_GROUND_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../camera_calibration/output/ground_projection.json"
    )
    add_test(NAME xtnetrc_visual_odometry_test
             COMMAND xtnetrc_visual_odometry_test)
endif()
