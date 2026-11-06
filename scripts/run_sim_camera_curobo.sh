#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
NEWTON_PY="${HOME}/newton_ws/.venv-cpu/bin/python"
CUROBO_PY="${HOME}/curobo_ros_env/bin/python"
EXCHANGE_FILE="$(mktemp /tmp/newton-camera-curobo-XXXXXX.npz)"

cleanup() {
  rm -f -- "${EXCHANGE_FILE}"
}
trap cleanup EXIT INT TERM

"${NEWTON_PY}" "${REPO_DIR}/scripts/newton_camera_static_capture.py" \
  --output "${EXCHANGE_FILE}"

set +u
source "${HOME}/use_curobo_env.sh"
set -u
export PYTHONPATH="${REPO_DIR}/curobo${PYTHONPATH:+:${PYTHONPATH}}"

cd "${REPO_DIR}"
"${CUROBO_PY}" scripts/curobo_camera_static_demo.py \
  --input "${EXCHANGE_FILE}" "$@"
