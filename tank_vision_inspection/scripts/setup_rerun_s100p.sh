#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
VENV_PATH="${PROJECT_ROOT}/.venv_rerun"

architecture="$(uname -m)"
if [[ "${architecture}" != "aarch64" && "${architecture}" != "arm64" ]]; then
  echo "ERROR: inspection_rerun requires S100P ARM64/aarch64; found ${architecture}" >&2
  exit 1
fi

python3 - <<'PY'
import sys
if sys.version_info < (3, 10):
    raise SystemExit(f"ERROR: Python >= 3.10 is required; found {sys.version.split()[0]}")
print(f"Python: {sys.version.split()[0]}")
PY

if [[ ! -x "${VENV_PATH}/bin/python" ]]; then
  python3 -m venv --system-site-packages "${VENV_PATH}"
fi

source "${VENV_PATH}/bin/activate"
python -m pip install "rerun-sdk==0.36.2"
python - <<'PY'
import rerun as rr
if rr.__version__ != "0.36.2":
    raise SystemExit(f"ERROR: expected rerun-sdk 0.36.2, found {rr.__version__}")
print(f"Rerun SDK: {rr.__version__}")
PY

echo "Environment ready: ${VENV_PATH}"
echo "Activate with: source ${VENV_PATH}/bin/activate"
