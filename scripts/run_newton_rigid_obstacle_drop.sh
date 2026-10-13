#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${HOME}/newton_ws/.venv-cpu/bin/python"
OUTPUT_DIR="${1:-${HOME}/ur5_ws/run_logs}"
TESTED_COMMIT="$(git -C "${REPO_DIR}" rev-parse HEAD)"

if [[ -n "$(git -C "${REPO_DIR}" status --porcelain --untracked-files=normal)" ]]; then
  echo "ERROR: refusing Step A result capture from a dirty git tree." >&2
  exit 3
fi

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "ERROR: missing ${PYTHON_BIN}; run scripts/bootstrap_school_jazzy.sh first." >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"
"${PYTHON_BIN}" "${REPO_DIR}/scripts/newton_rigid_obstacle_drop.py" \
  --config "${REPO_DIR}/config/obstacles.yaml" \
  --enable-obstacles \
  --tested-commit "${TESTED_COMMIT}" \
  --output "${OUTPUT_DIR}/rigid_obstacle_step_a0.json"

exec "${PYTHON_BIN}" "${REPO_DIR}/scripts/newton_fem_obstacle_drop.py" \
  --config "${REPO_DIR}/config/obstacles.yaml" \
  --enable-obstacles \
  --tested-commit "${TESTED_COMMIT}" \
  --output "${OUTPUT_DIR}/rigid_obstacle_step_a1_fem.json"
