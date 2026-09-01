cmake_minimum_required(VERSION 3.16)

project(xtnetrc_motion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 独立算法库：没有 OpenCV、pigpio 或其他硬件依赖，后续可链接到主程序。
add_library(xtnetrc_motion STATIC
    src/controller.cpp
    src/config_loader.cpp
)
target_include_directories(xtnetrc_motion PUBLIC include)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(xtnetrc_motion PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 离线演示只打印计算结果，不会访问 GPIO/PWM。
add_executable(xtnetrc_motion_demo src/demo.cpp)
target_link_libraries(xtnetrc_motion_demo PRIVATE xtnetrc_motion)

include(CTest)
if(BUILD_TESTING)
    add_executable(xtnetrc_motion_test tests/controller_test.cpp)
    target_link_libraries(xtnetrc_motion_test PRIVATE xtnetrc_motion)
    target_compile_definitions(xtnetrc_motion_test PRIVATE
        XTNETRC_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/motion_default.json")
    add_test(NAME xtnetrc_motion_test COMMAND xtnetrc_motion_test)
endif()
