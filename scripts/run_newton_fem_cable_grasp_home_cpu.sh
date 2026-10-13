#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export CUDA_VISIBLE_DEVICES=""
export GRASP_DEVICE="cpu"
export GRASP_FEM_CELLS_X="40"
export GRASP_FEM_CELLS_Y="2"
export GRASP_FEM_CELLS_Z="2"

echo "FEM_CABLE_RUNTIME_PROFILE home-cpu"
exec "${SCRIPT_DIR}/run_newton_fem_cable_grasp_jazzy.sh"
