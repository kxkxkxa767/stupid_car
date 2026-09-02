#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
build_dir="${project_root}/build/pi-aarch64-visual-distance-test"
sysroot="${project_root}/local/pi-sysroot"

cmake --fresh \
    -S "${project_root}/main/visual_distance_test" \
    -B "${build_dir}" \
    -DCMAKE_TOOLCHAIN_FILE="${project_root}/cmake/toolchains/raspberry-pi-aarch64.cmake" \
    -DXTNETRC_PI_SYSROOT="${sysroot}" \
    -DOpenCV_DIR="${sysroot}/usr/local/lib/cmake/opencv4" \
    -DXTNETRC_STATIC_CXX_RUNTIME=ON \
    -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE=Release

cmake --build "${build_dir}" --parallel
file "${build_dir}/xtnetrc_visual_distance_test"
