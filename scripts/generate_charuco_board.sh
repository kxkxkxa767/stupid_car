#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
bundled_python="/Users/yuhaojin/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3"

"${project_root}/.venv/bin/python" "${script_dir}/generate_charuco_board.py"
"${bundled_python}" "${script_dir}/make_charuco_board_pdf.py"
