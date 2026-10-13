#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export CUDA_VISIBLE_DEVICES=""
export GRASP_DEVICE="cpu"

echo "CABLE_RUNTIME_PROFILE home-cpu"
exec "${SCRIPT_DIR}/run_newton_cable_grasp_jazzy.sh"
