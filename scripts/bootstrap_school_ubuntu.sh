#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${HOME}/ur5_ws"
NEWTON_WS="${HOME}/newton_ws"
ROBOTIQ_DIR="${ROS_WS}/src/ros"
BRIDGE_DIR="${ROS_WS}/src/newton_ros_bridge"
LESSON_DIR="${NEWTON_WS}/lessons"
ASSET_DIR="${NEWTON_WS}/ros_bridge_assets"
ROBOTIQ_COMMIT="69ab9f4ed99a54cbf16ba5aa0b15ed3128fd1267"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
mkdir -p "${ROS_WS}/src" "${LESSON_DIR}/segmented_strip" "${ASSET_DIR}"

if [[ ! -d "${ROBOTIQ_DIR}/.git" ]]; then
  git clone https://github.com/robotiq/ros.git "${ROBOTIQ_DIR}"
fi
git -C "${ROBOTIQ_DIR}" fetch origin
git -C "${ROBOTIQ_DIR}" checkout "${ROBOTIQ_COMMIT}"

mkdir -p "${BRIDGE_DIR}"
cp -a "${REPO_DIR}/docs/experiments/newton-ros2-bridge/prototype/." "${BRIDGE_DIR}/"
find "${BRIDGE_DIR}" -type d -name __pycache__ -prune -exec rm -rf {} +

cp "${REPO_DIR}/docs/experiments/newton-robot-grasp/prototype/"*.py \
  "${LESSON_DIR}/segmented_strip/"

TOOLS_VENV="${NEWTON_WS}/.tools"
if [[ ! -x "${TOOLS_VENV}/bin/uv" ]]; then
  python3 -m venv "${TOOLS_VENV}"
  "${TOOLS_VENV}/bin/python" -m pip install --upgrade pip uv
fi
UV="${TOOLS_VENV}/bin/uv"
"${UV}" python install 3.12
"${UV}" venv --python 3.12 "${LESSON_DIR}/.venv-cpu"
"${UV}" pip install \
  --python "${LESSON_DIR}/.venv-cpu/bin/python" \
  -r "${REPO_DIR}/docs/experiments/newton-ros2-bridge/newton-school-requirements.txt"

cd "${ROS_WS}"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source "${ROS_WS}/install/setup.bash"

xacro "${REPO_DIR}/urdf/ur5_robotiq.urdf.xacro" \
  > "${ASSET_DIR}/ur5_robotiq.urdf"

UR_SHARE="$(ros2 pkg prefix --share ur_description)"
ROBOTIQ_SHARE="$(ros2 pkg prefix --share robotiq_description)"
export UR_SHARE ROBOTIQ_SHARE ASSET_DIR
python3 - <<'PY'
import os
from pathlib import Path

asset_dir = Path(os.environ["ASSET_DIR"])
source = asset_dir / "ur5_robotiq.urdf"
target = asset_dir / "ur5_robotiq.newton.urdf"
text = source.read_text(encoding="utf-8")
text = text.replace("package://ur_description/", os.environ["UR_SHARE"] + "/")
text = text.replace(
    "package://robotiq_description/", os.environ["ROBOTIQ_SHARE"] + "/"
)
target.write_text(text, encoding="utf-8")
if "package://" in text:
    raise SystemExit("ERROR: unresolved package:// URI remains in Newton URDF")
print(f"Generated: {target}")
PY

"${LESSON_DIR}/.venv-cpu/bin/python" - <<'PY'
from importlib.metadata import version
import newton
import numpy
import viser
print("Newton environment: OK")
print("newton", newton.__version__)
print("numpy", numpy.__version__)
print("viser", version("viser"))
PY

echo "School workspace bootstrap completed."
echo "Next: read docs/SCHOOL_COMPUTER_HANDOFF.md"
