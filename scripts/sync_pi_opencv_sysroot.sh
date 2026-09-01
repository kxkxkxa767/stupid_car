#!/usr/bin/env bash
set -euo pipefail

# 从树莓派只读同步现有 OpenCV 4.13 开发文件。
# 所有写入都发生在 Mac 的 local/pi-sysroot，不会修改树莓派。

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
pi_target="${1:-5G@192.168.124.5}"
sysroot="${project_root}/local/pi-sysroot"
control_dir="$(mktemp -d "${TMPDIR:-/tmp}/xtnetrc-ssh.XXXXXX")"
control_socket="${control_dir}/control"

cleanup() {
    ssh -S "${control_socket}" -O exit "${pi_target}" >/dev/null 2>&1 || true
    rmdir "${control_dir}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

# 复用同一条 SSH 连接，整个同步过程只需输入一次密码。
ssh -M -S "${control_socket}" -o ControlPersist=60 -fnNT "${pi_target}"
rsync_ssh="ssh -S ${control_socket}"

mkdir -p \
    "${sysroot}/usr/local/include" \
    "${sysroot}/usr/local/lib/cmake" \
    "${sysroot}/usr/local/lib/pkgconfig"

echo "同步 OpenCV 头文件……"
rsync -av -e "${rsync_ssh}" "${pi_target}:/usr/local/include/opencv4" \
    "${sysroot}/usr/local/include/"

echo "同步 OpenCV CMake 配置……"
rsync -av -e "${rsync_ssh}" "${pi_target}:/usr/local/lib/cmake/opencv4" \
    "${sysroot}/usr/local/lib/cmake/"

echo "同步 OpenCV pkg-config 配置……"
rsync -av -e "${rsync_ssh}" "${pi_target}:/usr/local/lib/pkgconfig/opencv4.pc" \
    "${sysroot}/usr/local/lib/pkgconfig/"

echo "同步 OpenCV 动态库及符号链接……"
rsync -av -e "${rsync_ssh}" --include='libopencv_*.so*' --exclude='*' \
    "${pi_target}:/usr/local/lib/" "${sysroot}/usr/local/lib/"

echo "同步这 5 个 OpenCV 组件实际使用的树莓派系统动态库……"
dependency_paths="$(
    ssh -S "${control_socket}" "${pi_target}" \
        "ldd /usr/local/lib/libopencv_core.so.4.13.0 \
             /usr/local/lib/libopencv_imgproc.so.4.13.0 \
             /usr/local/lib/libopencv_highgui.so.4.13.0 \
             /usr/local/lib/libopencv_videoio.so.4.13.0 \
             /usr/local/lib/libopencv_imgcodecs.so.4.13.0" \
    | awk '/=> \/|^\// { for (i = 1; i <= NF; ++i) if ($i ~ /^\// && $i !~ /:$/) print $i }' \
    | sort -u
)"

if [[ -z "${dependency_paths}" ]]; then
    echo "没有取得 OpenCV 系统依赖列表。" >&2
    exit 1
fi

# -L 解引用远端符号链接，按原绝对目录结构保存实际库文件。
printf '%s\n' "${dependency_paths}" \
    | rsync -avL -e "${rsync_ssh}" -R --files-from=- \
        "${pi_target}:/" "${sysroot}/"

echo "同步树莓派已有的 glibc 2.36 链接脚本和启动文件……"
glibc_development_paths="$(
    ssh -S "${control_socket}" "${pi_target}" \
        "dpkg -L libc6-dev:arm64 libc6:arm64" \
    | awk '/^\/usr\/lib\/aarch64-linux-gnu\/(crt[^/]*\.o|[^/]*\.(a|so))$/ ||
           /^\/lib\/(aarch64-linux-gnu\/)?ld-linux-aarch64\.so\.1$/ { print }' \
    | sort -u
)"

if [[ -z "${glibc_development_paths}" ]]; then
    echo "树莓派已有 libc6-dev，但没有取得链接文件列表。" >&2
    exit 1
fi

printf '%s\n' "${glibc_development_paths}" \
    | rsync -avL -e "${rsync_ssh}" -R --files-from=- \
        "${pi_target}:/" "${sysroot}/"

echo
echo "树莓派 OpenCV sysroot 已同步到：${sysroot}"
echo "该操作没有写入树莓派。"
