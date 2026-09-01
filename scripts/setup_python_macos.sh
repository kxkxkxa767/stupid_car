#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="${project_dir}/.venv"

if [[ ! -x "${venv_dir}/bin/python" ]]; then
    /usr/bin/python3 -m venv "${venv_dir}"
fi

"${venv_dir}/bin/python" -m pip install --upgrade pip
"${venv_dir}/bin/python" -m pip install --requirement "${project_dir}/requirements.txt"

"${venv_dir}/bin/python" - <<'PY'
import cv2
import numpy

print(f"Python OpenCV: {cv2.__version__}")
print(f"NumPy: {numpy.__version__}")
PY
