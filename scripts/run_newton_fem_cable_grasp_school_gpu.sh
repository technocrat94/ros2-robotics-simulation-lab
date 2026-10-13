#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi >/dev/null 2>&1; then
  echo "ERROR: school-gpu profile requires a working NVIDIA driver." >&2
  exit 1
fi

export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export GRASP_DEVICE="cuda:0"
export GRASP_FEM_CELLS_X="80"
export GRASP_FEM_CELLS_Y="2"
export GRASP_FEM_CELLS_Z="2"

echo "FEM_CABLE_RUNTIME_PROFILE school-gpu"
exec "${SCRIPT_DIR}/run_newton_fem_cable_grasp_jazzy.sh"
