#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
remote_host="${1:-5G@pi.local}"
remote_root="${2:-/home/5G/5G}"

deploy_dirs=(OpenCV PWM main imu tflite voice picsave)

printf '部署目标：%s:%s/\n' "$remote_host" "$remote_root"
printf '只合并新工程文件，不使用 --delete，不删除树莓派现有文件。\n'

for deploy_dir in "${deploy_dirs[@]}"; do
    rsync -av \
        --exclude='.DS_Store' \
        --exclude='__pycache__/' \
        --exclude='build/' \
        "${project_root}/${deploy_dir}/" \
        "${remote_host}:${remote_root}/${deploy_dir}/"
done

printf '部署完成。新模块位于 %s/main/vision、main/motion_control 和 main/camera_gimbal。\n' \
    "$remote_root"
