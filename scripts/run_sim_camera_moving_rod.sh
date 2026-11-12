#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CUROBO_PY="${HOME}/curobo_ros_env/bin/python"

set +u
source "${HOME}/use_curobo_env.sh"
set -u
export PYTHONPATH="${REPO_DIR}/curobo${PYTHONPATH:+:${PYTHONPATH}}"

cd "${REPO_DIR}"
"${CUROBO_PY}" scripts/curobo_camera_moving_rod.py "$@"
