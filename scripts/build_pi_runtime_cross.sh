#!/usr/bin/env bash
set -euo pipefail

# Local compilation only. Never connects to / changes the Raspberry Pi.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
build_dir="${project_root}/build/pi-aarch64-runtime"
pi_sysroot="${project_root}/local/pi-sysroot"

cmake -S "${project_root}" -B "${build_dir}" \
    -DCMAKE_TOOLCHAIN_FILE="${project_root}/cmake/toolchains/raspberry-pi-aarch64.cmake" \
    -DXTNETRC_PI_SYSROOT="${pi_sysroot}" \
    -DOpenCV_DIR="${pi_sysroot}/usr/local/lib/cmake/opencv4" \
    -DXTNETRC_BUILD_GUI=OFF -DXTNETRC_WITH_PIGPIO=ON \
    -DXTNETRC_STATIC_CXX_RUNTIME=ON -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build "${build_dir}" --parallel 6
file "${build_dir}/xtnetrc_car_runtime"
