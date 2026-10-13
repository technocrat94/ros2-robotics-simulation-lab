#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -ne 1 || ( "$1" != "c0" && "$1" != "c1" ) ]]; then
  echo "Usage: $0 c0|c1" >&2
  exit 2
fi

CASE="$1"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${HOME}/ur5_ws"
LOG_DIR="${ROS_WS}/run_logs"
PADDING="${MOVEIT_ROBOT_PADDING_M:-0.02}"
PLANNING_TIME="${MOVEIT_PLANNING_TIME_SECONDS:-10.0}"
TESTED_COMMIT="$(git -C "${REPO_DIR}" rev-parse HEAD)"
mkdir -p "${LOG_DIR}"

if [[ -n "$(git -C "${REPO_DIR}" status --porcelain --untracked-files=normal)" ]]; then
  echo "ERROR: refusing Step C result capture from a dirty git tree." >&2
  exit 3
fi
if [[ "${CASE}" == "c0" ]]; then
  PROFILE="step_c_cable_no_wall"
else
  PROFILE="step_c_cable_wall"
fi

MARKER="$(mktemp "${LOG_DIR}/step_c_${CASE}_marker.XXXXXX")"
trap 'rm -f "${MARKER}"' EXIT
export GRASP_SCENE_PROFILE="${PROFILE}"
export MOVEIT_ROBOT_PADDING_M="${PADDING}"
export MOVEIT_PLANNING_TIME_SECONDS="${PLANNING_TIME}"
export FEM_CABLE_SAFE_PREGRASP="false"

echo "STEP_C_PICK_CASE ${CASE}"
echo "GRASP_SCENE_PROFILE ${PROFILE}"
echo "MOVEIT_ROBOT_PADDING_M ${PADDING}"
echo "MOVEIT_PLANNING_TIME_SECONDS ${PLANNING_TIME}"
"${REPO_DIR}/scripts/run_jazzy_fem_cable_pick.sh"

PICK_LOG="$(find "${LOG_DIR}" -maxdepth 1 -type f \
  -name 'jazzy_fem_cable_pick_*.log' -newer "${MARKER}" \
  -print | sort | tail -1)"
if [[ -z "${PICK_LOG}" ]]; then
  echo "ERROR: no Step C MoveIt pick log was created." >&2
  exit 1
fi

ENDPOINT_JSON=""
for _ in $(seq 1 900); do
  ENDPOINT_JSON="$(find "${LOG_DIR}" -maxdepth 1 -type f \
    -name 'jazzy_fem_cable_endpoint_*.result.json' -newer "${MARKER}" \
    -print | sort | tail -1)"
  [[ -n "${ENDPOINT_JSON}" ]] && break
  sleep 1
done
if [[ -z "${ENDPOINT_JSON}" ]]; then
  echo "ERROR: Newton produced no result JSON within 900 seconds." >&2
  exit 1
fi

RESULT_JSON="${LOG_DIR}/step_c_${CASE}_$(date +%Y%m%d_%H%M%S).result.json"
python3 "${REPO_DIR}/scripts/extract_step_c_result.py" \
  --case "${CASE}" --pick-log "${PICK_LOG}" \
  --endpoint-json "${ENDPOINT_JSON}" --output "${RESULT_JSON}" \
  --tested-commit "${TESTED_COMMIT}" --profile "${PROFILE}" \
  --padding "${PADDING}" --planning-time "${PLANNING_TIME}"
echo "STEP_C_PICK_LOG ${PICK_LOG}"
echo "STEP_C_ENDPOINT_RESULT_JSON ${ENDPOINT_JSON}"
echo "STEP_C_COMBINED_RESULT_JSON ${RESULT_JSON}"
