#!/usr/bin/env bash
set -euo pipefail

# 在 Mac 上生成可供 Debian 12 / ARM64 树莓派运行的运动控制程序。
# 本脚本只读取 main/motion_control，不会连接或修改树莓派。

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
build_dir="${project_root}/build/pi-aarch64-motion"
toolchain_file="${project_root}/cmake/toolchains/raspberry-pi-aarch64.cmake"

for command_name in cmake aarch64-unknown-linux-gnu-g++; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "缺少命令：${command_name}" >&2
        echo "请先安装 ARM64 Linux 交叉编译工具链。" >&2
        exit 1
    fi
done

cmake \
    -S "${project_root}/main/motion_control" \
    -B "${build_dir}" \
    -DCMAKE_TOOLCHAIN_FILE="${toolchain_file}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF

cmake --build "${build_dir}" --parallel

binary="${build_dir}/xtnetrc_motion_demo"
echo
echo "交叉编译完成：${binary}"
file "${binary}"

