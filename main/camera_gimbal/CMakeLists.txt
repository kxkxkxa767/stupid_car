cmake_minimum_required(VERSION 3.16)

project(xtnetrc_camera_gimbal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 控制核心不依赖 OpenCV。pigpio 通过 dlopen 在树莓派运行时加载，因此
# Mac 编译和 ARM64 交叉编译都不需要复制或重新安装 pigpio 开发包。
add_library(xtnetrc_camera_gimbal STATIC
    src/gimbal.cpp
    src/config_loader.cpp
    src/pigpio_driver.cpp
)
target_include_directories(xtnetrc_camera_gimbal PUBLIC include)
target_link_libraries(xtnetrc_camera_gimbal PRIVATE ${CMAKE_DL_LIBS})

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(xtnetrc_camera_gimbal PRIVATE
        -Wall -Wextra -Wpedantic)
endif()

# 命令行工具用于树莓派单独验证；Mac 上可以编译，但因没有 libpigpio
# 会在启动硬件前给出明确错误，不会访问任何本机 GPIO。
add_executable(xtnetrc_gimbal_cli src/cli.cpp)
target_link_libraries(xtnetrc_gimbal_cli PRIVATE xtnetrc_camera_gimbal)

include(CTest)
if(BUILD_TESTING)
    add_executable(xtnetrc_camera_gimbal_test tests/gimbal_test.cpp)
    target_link_libraries(xtnetrc_camera_gimbal_test PRIVATE
        xtnetrc_camera_gimbal)
    target_compile_definitions(xtnetrc_camera_gimbal_test PRIVATE
        XTNETRC_GIMBAL_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/gimbal_default.json")
    add_test(NAME xtnetrc_camera_gimbal_test
             COMMAND xtnetrc_camera_gimbal_test)
endif()
