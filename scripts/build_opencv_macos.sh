#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "$0")/.." && pwd)"
opencv_version="${OPENCV_VERSION:-4.14.0}"
archive="$project_root/third_party/opencv-${opencv_version}.tar.gz"
source_dir="$project_root/third_party/opencv-${opencv_version}"
build_dir="$project_root/build/opencv-${opencv_version}"
install_dir="$project_root/local/opencv"

mkdir -p "$project_root/third_party" "$build_dir" "$install_dir"

if [[ ! -f "$archive" ]]; then
  curl --fail --location \
    "https://github.com/opencv/opencv/archive/refs/tags/${opencv_version}.tar.gz" \
    --output "$archive"
fi

if [[ ! -d "$source_dir" ]]; then
  tar -xzf "$archive" -C "$project_root/third_party"
fi

cmake \
  -S "$source_dir" \
  -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$install_dir" \
  -DBUILD_LIST=core,imgproc,imgcodecs,video,videoio,highgui \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_opencv_apps=OFF \
  -DBUILD_opencv_python2=OFF \
  -DBUILD_opencv_python3=OFF \
  -DBUILD_JAVA=OFF \
  -DWITH_OPENCL=OFF

cmake --build "$build_dir" --parallel "$(sysctl -n hw.ncpu)"
cmake --install "$build_dir"

printf 'OpenCV %s installed in %s\n' "$opencv_version" "$install_dir"
