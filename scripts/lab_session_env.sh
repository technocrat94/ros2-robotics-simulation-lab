#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Source this file: source scripts/lab_session_env.sh" >&2
  exit 1
fi

# Derive stable per-user defaults from the Linux UID. They can be overridden
# when the laboratory assigns explicit values.
_robotics_slot=$((UID % 1000))
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((20 + UID % 200))}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export NEWTON_STATE_PORT="${NEWTON_STATE_PORT:-$((20000 + 2 * _robotics_slot))}"
export NEWTON_COMMAND_PORT="${NEWTON_COMMAND_PORT:-$((20001 + 2 * _robotics_slot))}"
export NEWTON_VIEWER_PORT="${NEWTON_VIEWER_PORT:-$((30000 + _robotics_slot))}"
unset _robotics_slot

echo "Laboratory isolation environment"
echo "  user=${USER} uid=${UID}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "  Newton state=${NEWTON_STATE_PORT} command=${NEWTON_COMMAND_PORT} viewer=${NEWTON_VIEWER_PORT}"
