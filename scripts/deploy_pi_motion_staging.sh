#!/usr/bin/env bash
set -euo pipefail

# 只把运动控制测试程序上传到独立 staging 目录。
# 默认目的地不在 /home/5G/5G 内，因此不会覆盖原车视觉识别代码。

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
binary="${project_root}/build/pi-aarch64-motion/xtnetrc_motion_demo"
config="${project_root}/main/motion_control/config/motion_default.json"
pi_target="${1:-5G@192.168.124.5}"
remote_dir="${2:-/home/5G/xtnetrc_staging/motion_control}"

if [[ ! -x "${binary}" ]]; then
    echo "尚未找到交叉编译结果：${binary}" >&2
    echo "请先运行 scripts/build_pi_motion_cross.sh" >&2
    exit 1
fi

ssh "${pi_target}" "mkdir -p '${remote_dir}'"
rsync -av --progress "${binary}" "${config}" "${pi_target}:${remote_dir}/"

echo
echo "已上传到独立测试目录：${pi_target}:${remote_dir}"
echo "运行命令：ssh ${pi_target} \"cd '${remote_dir}' && ./xtnetrc_motion_demo --config ./motion_default.json left\""
