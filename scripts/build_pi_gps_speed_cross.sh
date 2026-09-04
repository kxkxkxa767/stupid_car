#!/usr/bin/env bash
set -euo pipefail

# 在 Mac 上构建 Debian 12 / ARM64 只读 GPS/IMU 诊断程序。

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
build_dir="${project_root}/build/pi-aarch64-gps-speed"
toolchain_file="${project_root}/cmake/toolchains/raspberry-pi-aarch64.cmake"

for command_name in cmake aarch64-unknown-linux-gnu-g++; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "缺少命令：${command_name}" >&2
        exit 1
    fi
done

cmake \
    -S "${project_root}/main/gps_speed_control" \
    -B "${build_dir}" \
    -DCMAKE_TOOLCHAIN_FILE="${toolchain_file}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF
cmake --build "${build_dir}" --parallel

binary="${build_dir}/xtnetrc_gps_speed_monitor"
echo "交叉编译完成：${binary}"
file "${binary}"
