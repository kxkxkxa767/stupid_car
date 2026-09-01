#!/usr/bin/env bash
set -euo pipefail

# 在 Mac 上生成可供 Debian 12 / ARM64 树莓派运行的云台控制程序。
# pigpio 在树莓派运行时动态加载，交叉编译阶段无需下载它的头文件或库。

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
build_dir="${project_root}/build/pi-aarch64-gimbal"
toolchain_file="${project_root}/cmake/toolchains/raspberry-pi-aarch64.cmake"

for command_name in cmake aarch64-unknown-linux-gnu-g++; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "缺少命令：${command_name}" >&2
        echo "请先安装 ARM64 Linux 交叉编译工具链。" >&2
        exit 1
    fi
done

cmake \
    -S "${project_root}/main/camera_gimbal" \
    -B "${build_dir}" \
    -DCMAKE_TOOLCHAIN_FILE="${toolchain_file}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF

cmake --build "${build_dir}" --parallel

binary="${build_dir}/xtnetrc_gimbal_cli"
echo
echo "交叉编译完成：${binary}"
file "${binary}"
