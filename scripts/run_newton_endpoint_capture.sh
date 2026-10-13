#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -ne 4 ]]; then
  echo "Usage: $0 PYTHON ENDPOINT LOG DEVICE" >&2
  exit 2
fi

PYTHON_BIN="$1"
ENDPOINT="$2"
ENDPOINT_LOG="$3"
EXPECTED_DEVICE="$4"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RESULT_JSON="${ENDPOINT_LOG%.log}.result.json"
RESULT_TIMEOUT_SECONDS="${NEWTON_RESULT_TIMEOUT_SECONDS:-1800}"

if [[ -n "$(git -C "${REPO_DIR}" status --porcelain --untracked-files=normal)" ]]; then
  echo "ERROR: refusing to capture a result from a dirty git tree." >&2
  exit 3
fi

if ! python3 "${SCRIPT_DIR}/check_newton_runtime_sync.py" \
  --repo "${REPO_DIR}" \
  --bridge-runtime "${HOME}/ur5_ws/src/newton_ros_bridge/newton_moveit_grasp_endpoint.py" \
  --lesson-runtime "${HOME}/newton_ws/lessons/segmented_strip"; then
  echo "ERROR: refusing to capture a result from runtime files that differ from the repository." >&2
  exit 3
fi

TESTED_COMMIT="$(git -C "${REPO_DIR}" rev-parse HEAD)"

export NEWTON_TESTED_COMMIT="${TESTED_COMMIT}"
export NEWTON_AUTO_RESULT_AFTER_SEQUENCE="1"
export NEWTON_RESULT_SETTLE_SIM_SECONDS="${NEWTON_RESULT_SETTLE_SIM_SECONDS:-1.0}"

echo "NEWTON_TESTED_COMMIT ${TESTED_COMMIT}"
echo "NEWTON_RESULT_TIMEOUT_SECONDS ${RESULT_TIMEOUT_SECONDS}"
echo "NEWTON_RESULT_JSON ${RESULT_JSON}"

set +e
timeout --signal=INT --kill-after=30s "${RESULT_TIMEOUT_SECONDS}" \
  "${PYTHON_BIN}" "${ENDPOINT}" 2>&1 | tee "${ENDPOINT_LOG}"
status=${PIPESTATUS[0]}
set -e

if [[ "${status}" -eq 124 ]]; then
  echo "ERROR: Newton endpoint timed out after ${RESULT_TIMEOUT_SECONDS}s before a complete result." >&2
  exit 124
fi
if [[ "${status}" -ne 0 ]]; then
  echo "ERROR: Newton endpoint exited with status ${status}." >&2
  exit "${status}"
fi

python3 "${SCRIPT_DIR}/extract_moveit_grasp_result.py" \
  --log "${ENDPOINT_LOG}" \
  --output "${RESULT_JSON}" \
  --tested-commit "${TESTED_COMMIT}" \
  --device "${EXPECTED_DEVICE}"
