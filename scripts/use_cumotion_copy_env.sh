#!/usr/bin/env bash

# Load the lab's existing cuMotion/ROS environment first, then override only
# the cuRobo source path so JIT extensions are built in our independent copy.
source "${HOME}/use_cumotion_rviz_env.sh"

export CUROBO_COPY_PATH="${HOME}/cumotion_copy/curobo/src"
export PYTHONPATH="${CUROBO_COPY_PATH}:${PYTHONPATH}"
export TORCH_CUDA_ARCH_LIST="8.6"
