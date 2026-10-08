#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${HOME}/ur5_ws"
NEWTON_WS="${HOME}/newton_ws"
BRIDGE_DIR="${ROS_WS}/src/newton_ros_bridge"
LESSON_DIR="${NEWTON_WS}/lessons/segmented_strip"
ASSET_DIR="${NEWTON_WS}/ros_bridge_assets"
VENV_DIR="${NEWTON_WS}/.venv-cpu"
BUILD_DIR="${ROS_WS}/build_jazzy_port"
INSTALL_DIR="${ROS_WS}/install_jazzy_port"
LOG_DIR="${ROS_WS}/log_jazzy_port"
REQUIREMENTS="${REPO_DIR}/docs/experiments/newton-ros2-bridge/newton-school-requirements.txt"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: ROS 2 Jazzy was not found at /opt/ros/jazzy." >&2
  exit 1
fi
if [[ "${REPO_DIR}" != "${ROS_WS}/src/ur5_moveit_demo" ]]; then
  echo "ERROR: expected repository at ${ROS_WS}/src/ur5_moveit_demo; got ${REPO_DIR}." >&2
  exit 1
fi
if ! command -v python3.12 >/dev/null 2>&1; then
  echo "ERROR: python3.12 is required for the Newton 1.5.1 environment." >&2
  exit 1
fi

source "${REPO_DIR}/scripts/lab_session_env.sh"
source /opt/ros/jazzy/setup.bash
if [[ -f "${INSTALL_DIR}/setup.bash" ]]; then
  source "${INSTALL_DIR}/setup.bash"
fi
set -u

mkdir -p "${BRIDGE_DIR}" "${LESSON_DIR}" "${ASSET_DIR}"
cp -a "${REPO_DIR}/docs/experiments/newton-ros2-bridge/prototype/." "${BRIDGE_DIR}/"
cp -a "${REPO_DIR}/docs/experiments/newton-robot-grasp/prototype/." "${LESSON_DIR}/"
cp -a "${REPO_DIR}/docs/experiments/newton-soft-strip/prototype/." "${LESSON_DIR}/"
find "${BRIDGE_DIR}" "${LESSON_DIR}" -type d -name __pycache__ -prune -exec rm -rf {} +

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  python3.12 -m venv "${VENV_DIR}"
fi
"${VENV_DIR}/bin/python" -m pip install -r "${REQUIREMENTS}"

cd "${ROS_WS}"
colcon --log-base "${LOG_DIR}" build \
  --build-base "${BUILD_DIR}" \
  --install-base "${INSTALL_DIR}" \
  --symlink-install \
  --packages-select \
    robotiq_controllers \
    robotiq_description \
    ur5_moveit_demo \
    newton_ros_bridge
set +u
source "${INSTALL_DIR}/setup.bash"
set -u

xacro "${REPO_DIR}/urdf/ur5_robotiq.urdf.xacro" \
  name:=ur ur_type:=ur5 use_fake_hardware:=true \
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
if "package://" in text:
    raise SystemExit("ERROR: unresolved package:// URI remains in Newton URDF")
if "/opt/ros/humble/" in text:
    raise SystemExit("ERROR: Humble path remains in Newton URDF")
target.write_text(text, encoding="utf-8")

import xml.etree.ElementTree as ET

root = ET.fromstring(text)
mesh_paths = sorted({
    element.attrib["filename"]
    for element in root.iter("mesh")
    if "filename" in element.attrib
})
if not mesh_paths:
    raise SystemExit("ERROR: generated Newton URDF contains no mesh paths")
missing = [path for path in mesh_paths if not Path(path).is_file()]
if missing:
    raise SystemExit("ERROR: missing Newton URDF meshes:\\n" + "\\n".join(missing))
print(f"Generated: {target}")
print(f"Verified mesh paths: {len(mesh_paths)}")
PY

CUDA_VISIBLE_DEVICES="" "${VENV_DIR}/bin/python" - <<'PY'
from importlib.metadata import version
import newton
import numpy
import viser
import warp

print("Newton Jazzy environment: OK")
print("newton", newton.__version__)
print("numpy", numpy.__version__)
print("viser", version("viser"))
print("warp", warp.__version__)
PY

ros2 pkg executables newton_ros_bridge

echo "School Jazzy Newton workspace bootstrap completed."
echo "Read docs/jazzy/STATUS.md for the verified launch sequence."
