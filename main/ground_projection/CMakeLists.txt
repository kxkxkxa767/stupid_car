cmake_minimum_required(VERSION 3.20)
project(xtnetrc_ground_projection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core)
add_library(xtnetrc_ground_projection STATIC src/ground_projector.cpp)
target_include_directories(xtnetrc_ground_projection PUBLIC include)
target_link_libraries(xtnetrc_ground_projection PUBLIC ${OpenCV_LIBS})
target_compile_options(xtnetrc_ground_projection PRIVATE
    $<$<CXX_COMPILER_ID:AppleClang,Clang,GNU>:-Wall;-Wextra;-Wpedantic>
)

include(CTest)
if(BUILD_TESTING)
    add_executable(xtnetrc_ground_projection_test tests/ground_projector_test.cpp)
    target_link_libraries(xtnetrc_ground_projection_test PRIVATE
        xtnetrc_ground_projection)
    target_compile_definitions(xtnetrc_ground_projection_test PRIVATE
        XTNETRC_GROUND_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/ground_projection_test.json"
        XTNETRC_REAL_GROUND_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../camera_calibration/output/ground_projection.json")
    add_test(NAME xtnetrc_ground_projection_test
             COMMAND xtnetrc_ground_projection_test)
endif()
