#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PADDING="${MOVEIT_ROBOT_PADDING_M:-0.02}"
TESTED_COMMIT="$(git -C "${REPO_DIR}" rev-parse HEAD)"
LOG_DIR="${HOME}/ur5_ws/run_logs"
mkdir -p "${LOG_DIR}"
OUTPUT="${LOG_DIR}/step_c_geometry_$(date +%Y%m%d_%H%M%S).result.json"

if [[ -n "$(git -C "${REPO_DIR}" status --porcelain --untracked-files=normal)" ]]; then
  echo "ERROR: refusing Step C geometry evidence from a dirty git tree." >&2
  exit 3
fi
if ! python3 "${REPO_DIR}/scripts/check_newton_runtime_sync.py" \
  --repo "${REPO_DIR}" \
  --bridge-runtime "${HOME}/ur5_ws/src/newton_ros_bridge/newton_moveit_grasp_endpoint.py" \
  --lesson-runtime "${HOME}/newton_ws/lessons/segmented_strip"; then
  echo "ERROR: runtime sources differ from the tested repository." >&2
  exit 3
fi

CUDA_VISIBLE_DEVICES="" "${HOME}/newton_ws/.venv-cpu/bin/python" \
  "${REPO_DIR}/scripts/step_c_geometry_preflight.py" \
  --repo "${REPO_DIR}" --profile step_c_cable_wall \
  --padding "${PADDING}" --tested-commit "${TESTED_COMMIT}" \
  --output "${OUTPUT}"
echo "STEP_C_GEOMETRY_RESULT_JSON ${OUTPUT}"
