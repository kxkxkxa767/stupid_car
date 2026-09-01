#!/usr/bin/env bash
set -euo pipefail

# 只把交叉编译视觉程序上传到独立 staging 目录。
# 不会替换 /home/5G/5G 或树莓派现有视觉识别程序。

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
binary="${project_root}/build/pi-aarch64-vision/xt_netrc_vision"
pi_target="${1:-5G@192.168.124.5}"
remote_dir="${2:-/home/5G/xtnetrc_staging/vision_cross}"

if [[ ! -x "${binary}" ]]; then
    echo "尚未找到交叉编译结果：${binary}" >&2
    echo "请先运行 scripts/build_pi_vision_cross.sh" >&2
    exit 1
fi

ssh "${pi_target}" "mkdir -p '${remote_dir}'"
rsync -av --progress "${binary}" "${pi_target}:${remote_dir}/"

echo
echo "已上传到独立测试目录：${pi_target}:${remote_dir}"
echo "安全验证：ssh ${pi_target} '${remote_dir}/xt_netrc_vision --help'"

