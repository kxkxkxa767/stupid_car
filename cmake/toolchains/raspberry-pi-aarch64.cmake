# XT-NetRC 树莓派 64 位 Linux 交叉编译配置。
#
# 该文件只描述目标平台和编译器，不引用树莓派上的现有工程目录，
# 因此不会修改或覆盖原车视觉程序。

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

find_program(XTNETRC_AARCH64_CC aarch64-unknown-linux-gnu-gcc REQUIRED)
find_program(XTNETRC_AARCH64_CXX aarch64-unknown-linux-gnu-g++ REQUIRED)

set(CMAKE_C_COMPILER "${XTNETRC_AARCH64_CC}")
set(CMAKE_CXX_COMPILER "${XTNETRC_AARCH64_CXX}")

# Homebrew 工具链自带 Linux ARM64 sysroot（glibc 2.28 基线）。
# 不在这里写死 Cellar 版本号，升级工具链后仍可继续使用。
execute_process(
    COMMAND "${CMAKE_CXX_COMPILER}" -print-sysroot
    OUTPUT_VARIABLE XTNETRC_COMPILER_SYSROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY
)
set(CMAKE_FIND_ROOT_PATH "${XTNETRC_COMPILER_SYSROOT}")

# 视觉交叉编译时可额外指定从实车只读同步的 OpenCV sysroot。
# 不设置该变量时，运动控制模块仍只使用编译器自带 sysroot。
if(DEFINED XTNETRC_PI_SYSROOT AND NOT XTNETRC_PI_SYSROOT STREQUAL "")
    list(PREPEND CMAKE_FIND_ROOT_PATH "${XTNETRC_PI_SYSROOT}")
    list(PREPEND CMAKE_PREFIX_PATH "${XTNETRC_PI_SYSROOT}/usr/local")

    # glibc 2.34 起启动 ABI 有变化；视觉程序必须让 GCC 同时使用
    # 树莓派 glibc 2.36 的 crt1/crti/crtn，不能混用工具链的 2.28 文件。
    set(XTNETRC_PI_STARTFILE_PREFIX
        "-B${XTNETRC_PI_SYSROOT}/usr/lib/aarch64-linux-gnu/")
    set(CMAKE_C_FLAGS_INIT "${XTNETRC_PI_STARTFILE_PREFIX}")
    set(CMAKE_CXX_FLAGS_INIT "${XTNETRC_PI_STARTFILE_PREFIX}")
endif()

# 构建时仍在 Mac 上运行 cmake 等工具，但头文件和库只能从目标 sysroot 查找。
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# 把 GCC 15 的 C++/GCC 运行库放进可执行文件，避免树莓派 GCC 12 的
# libstdc++ 缺少新版 GLIBCXX 符号。glibc 仍动态链接到树莓派系统版本。
option(XTNETRC_STATIC_CXX_RUNTIME
    "把交叉编译器的 libstdc++/libgcc 静态放入目标文件" ON)

set(XTNETRC_TARGET_LINKER_FLAGS "")
if(XTNETRC_STATIC_CXX_RUNTIME)
    string(APPEND XTNETRC_TARGET_LINKER_FLAGS " -static-libstdc++ -static-libgcc")
endif()
if(DEFINED XTNETRC_PI_SYSROOT AND NOT XTNETRC_PI_SYSROOT STREQUAL "")
    string(APPEND XTNETRC_TARGET_LINKER_FLAGS
        " -Wl,--sysroot=${XTNETRC_PI_SYSROOT}"
        " -L${XTNETRC_PI_SYSROOT}/lib/aarch64-linux-gnu"
        " -L${XTNETRC_PI_SYSROOT}/usr/lib/aarch64-linux-gnu"
        " -Wl,-rpath-link,${XTNETRC_PI_SYSROOT}/usr/local/lib"
        " -Wl,-rpath-link,${XTNETRC_PI_SYSROOT}/lib/aarch64-linux-gnu"
        " -Wl,-rpath-link,${XTNETRC_PI_SYSROOT}/usr/lib/aarch64-linux-gnu")
endif()
set(CMAKE_EXE_LINKER_FLAGS_INIT "${XTNETRC_TARGET_LINKER_FLAGS}")
