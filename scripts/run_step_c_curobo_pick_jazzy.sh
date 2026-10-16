#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${HOME}/ur5_ws"
LOG_DIR="${ROS_WS}/run_logs"
TESTED_COMMIT="$(git -C "${REPO_DIR}" rev-parse HEAD)"
mkdir -p "${LOG_DIR}"

if [[ -n "$(git -C "${REPO_DIR}" status --porcelain --untracked-files=normal)" ]]; then
  echo "ERROR: refusing cuRobo Step C evidence from a dirty git tree." >&2
  exit 3
fi

MARKER="$(mktemp "${LOG_DIR}/curobo_step_c_marker.XXXXXX")"
trap 'rm -f "${MARKER}"' EXIT

"${REPO_DIR}/scripts/run_curobo_step_c_pick_jazzy.sh"

PICK_LOG="$(find "${LOG_DIR}" -maxdepth 1 -type f \
  -name 'curobo_step_c_pick_*.log' -newer "${MARKER}" \
  -print | sort | tail -1)"
if [[ -z "${PICK_LOG}" ]]; then
  echo "ERROR: no cuRobo Step C pick log was created." >&2
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

RESULT_JSON="${LOG_DIR}/curobo_step_c_c1_$(date +%Y%m%d_%H%M%S).result.json"
python3 "${REPO_DIR}/scripts/extract_curobo_step_c_result.py" \
  --pick-log "${PICK_LOG}" \
  --endpoint-json "${ENDPOINT_JSON}" \
  --output "${RESULT_JSON}" \
  --tested-commit "${TESTED_COMMIT}"

echo "CUROBO_STEP_C_PICK_LOG ${PICK_LOG}"
echo "CUROBO_STEP_C_ENDPOINT_RESULT_JSON ${ENDPOINT_JSON}"
echo "CUROBO_STEP_C_COMBINED_RESULT_JSON ${RESULT_JSON}"
