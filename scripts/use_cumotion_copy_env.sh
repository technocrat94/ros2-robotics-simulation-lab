#!/usr/bin/env bash

# Load the lab's existing cuMotion/ROS environment first, then override only
# the cuRobo source path so JIT extensions are built in our independent copy.
source "${HOME}/use_cumotion_rviz_env.sh"

export CUROBO_COPY_PATH="${HOME}/cumotion_copy/curobo/src"
export ISAAC_ROS_CUMOTION_PY="${HOME}/cumotion_copy/isaac_ros_cumotion_py"
export PYTHONPATH="${ISAAC_ROS_CUMOTION_PY}:${CUROBO_COPY_PATH}:${PYTHONPATH}"
export TORCH_CUDA_ARCH_LIST="8.6"
# The copied checkout has no usable git metadata; v0.7.7 is the tag of the source it was copied from.
export SETUPTOOLS_SCM_PRETEND_VERSION=0.7.7
