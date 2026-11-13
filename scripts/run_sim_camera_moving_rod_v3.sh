#!/usr/bin/env bash
set -euo pipefail
REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${REPO_DIR}/scripts/lab_session_env.sh"
set +u
source "${HOME}/use_curobo_env.sh"
set -u
export PYTHONPATH="${REPO_DIR}/curobo${PYTHONPATH:+:${PYTHONPATH}}"
cd "${REPO_DIR}"
"${HOME}/curobo_ros_env/bin/python" scripts/curobo_moving_rod_v3.py "$@"
