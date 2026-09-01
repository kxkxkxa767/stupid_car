#!/usr/bin/env bash
set -euo pipefail

# 交叉编译摄像头 + 运动控制安全验证器；程序不包含 GPIO/PWM 输出。
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
build_dir="${project_root}/build/pi-aarch64-camera-motion-validation"
sysroot="${project_root}/local/pi-sysroot"

cmake --fresh \
    -S "${project_root}/main/camera_motion_validation" \
    -B "${build_dir}" \
    -DCMAKE_TOOLCHAIN_FILE="${project_root}/cmake/toolchains/raspberry-pi-aarch64.cmake" \
    -DXTNETRC_PI_SYSROOT="${sysroot}" \
    -DOpenCV_DIR="${sysroot}/usr/local/lib/cmake/opencv4" \
    -DXTNETRC_STATIC_CXX_RUNTIME=ON \
    -DCMAKE_BUILD_TYPE=Release

cmake --build "${build_dir}" --parallel
file "${build_dir}/xtnetrc_camera_motion_validation"
