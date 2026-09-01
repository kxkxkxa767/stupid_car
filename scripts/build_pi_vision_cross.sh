#!/usr/bin/env bash
set -euo pipefail

# 在 Mac 上使用实车 OpenCV sysroot 生成 Linux ARM64 视觉程序。
# 只生成本地文件，不会上传或替换树莓派上的现有视觉代码。

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
build_dir="${project_root}/build/pi-aarch64-vision"
sysroot="${project_root}/local/pi-sysroot"
opencv_config="${sysroot}/usr/local/lib/cmake/opencv4/OpenCVConfig.cmake"
toolchain_file="${project_root}/cmake/toolchains/raspberry-pi-aarch64.cmake"

if [[ ! -f "${opencv_config}" ]]; then
    echo "缺少树莓派 OpenCV sysroot：${opencv_config}" >&2
    echo "请先运行：./scripts/sync_pi_opencv_sysroot.sh 5G@192.168.124.5" >&2
    exit 1
fi

cmake --fresh \
    -S "${project_root}/main/vision" \
    -B "${build_dir}" \
    -DCMAKE_TOOLCHAIN_FILE="${toolchain_file}" \
    -DXTNETRC_PI_SYSROOT="${sysroot}" \
    -DOpenCV_DIR="${sysroot}/usr/local/lib/cmake/opencv4" \
    -DXTNETRC_STATIC_CXX_RUNTIME=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF

cmake --build "${build_dir}" --parallel

binary="${build_dir}/xt_netrc_vision"
echo
echo "视觉交叉编译完成：${binary}"
file "${binary}"
