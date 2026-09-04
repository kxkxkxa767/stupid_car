#!/usr/bin/env bash
set -euo pipefail

# 只部署到 staging；不读取、不编译、不覆盖 /home/5G/5G。

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
binary="${project_root}/build/pi-aarch64-gps-speed/xtnetrc_gps_speed_monitor"
pi_target="${1:-5G@192.168.107.252}"
remote_dir="${2:-/home/5G/xtnetrc_staging/gps_speed_control}"

if [[ ! -x "${binary}" ]]; then
    echo "尚未找到交叉编译结果：${binary}" >&2
    echo "请先运行 scripts/build_pi_gps_speed_cross.sh" >&2
    exit 1
fi

ssh "${pi_target}" "mkdir -p '${remote_dir}'"
rsync -av --progress "${binary}" "${pi_target}:${remote_dir}/"

echo "已部署只读程序：${pi_target}:${remote_dir}/xtnetrc_gps_speed_monitor"
