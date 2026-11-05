# Lab 3D camera and cuRobo survey

Survey date: 2026-11-05 (Asia/Taipei)

Scope: read-only inventory. No packages, drivers, or system settings were changed, and no
simulation was started.

## Summary

- The host is Ubuntu 24.04.4 LTS (`x86_64`) with ROS 2 Jazzy installed.
- The GPU is an NVIDIA GeForce RTX 3080 with 10 GiB VRAM. The NVIDIA driver is
  `580.178.04`, and `nvidia-smi` reports CUDA compatibility `13.0`.
- Intel RealSense command-line and GUI tools and the ROS RealSense packages are installed,
  but no 3D camera was connected or enumerated during this survey.
- `nvblox_msgs` is installed. No ROS package matching `cumotion`, `isaac`, or `curobo` was
  found in the sourced Jazzy environment.
- A local cuRobo source checkout is imported by the project environment. It identifies itself
  as `0.8.0.post1.dev43`.
- Docker cannot be queried by this user without `sudo`, so Isaac ROS/curobo container images
  and running containers could not be inventoried.

## 1. Machine

Commands were run from a normal user shell.

| Item | Observed fact |
| --- | --- |
| OS | Ubuntu 24.04.4 LTS (Noble), from `lsb_release -a` |
| Architecture | `x86_64`, from `uname -m` |
| Initial `ROS_DISTRO` | Empty/not set |
| Installed ROS distributions | `/opt/ros/jazzy` only |
| GPU | NVIDIA GeForce RTX 3080 |
| NVIDIA driver | `580.178.04` |
| CUDA reported by `nvidia-smi` | `13.0` (driver compatibility level, not necessarily the installed toolkit version) |
| GPU memory | `10240 MiB` total; `852 MiB` in use at survey time |

The project helper `~/use_curobo_env.sh` points cuRobo at a CUDA 12.8 toolkit under
`/usr/local/cuda-12.8`; this is separate from the CUDA compatibility level printed by
`nvidia-smi`.

## 2. Camera and USB inventory

`lsusb` showed only root hubs and the following peripherals:

- SiGma Micro XM102K optical mouse (`1c4f:0034`)
- Genesys Logic hub (`05e3:0608`)
- Intel AX201 Bluetooth (`8087:0026`)
- MSI Mystic Light (`1462:b916`)
- China Resource Semico keyboard (`1a2c:2124`)

Camera observations:

- USB camera: **not found**.
- `/dev/video*`: **not found**.
- `rs-enumerate-devices -s`: `No device detected. Is it plugged in?`
- RealSense tools on `PATH`:
  - `/usr/local/bin/rs-enumerate-devices`
  - `/usr/local/bin/realsense-viewer`
  - `rs-hello-realsense`
- ZED, Orbbec, Azure Kinect/Kinect, and DepthAI vendor tools on `PATH`: **not found**.

The device probes were repeated with direct host-device visibility because the restricted
execution environment initially hid libusb, udev, and NVIDIA devices. The facts above are
from the direct read-only host probes.

## 3. ROS packages

After sourcing `/opt/ros/jazzy/setup.bash`, this filter was applied to `ros2 pkg list`:

```text
realsense|zed|orbbec|azure|kinect|nvblox|cumotion|isaac|curobo
```

Matches:

```text
nvblox_msgs
realsense2_camera
realsense2_camera_msgs
```

ZED, Orbbec, Azure Kinect/Kinect, cuMotion, other Isaac ROS packages, and cuRobo ROS
packages: **not found** in that sourced environment.

## 4. Isaac ROS and Docker

Both `docker images` and `docker ps` failed with permission denied on
`unix:///var/run/docker.sock`. Per the read-only/no-sudo constraint, no elevated Docker query
was attempted. Therefore:

- Isaac/nvblox/curobo Docker images: **not determined**.
- Running Isaac/nvblox/curobo containers: **not determined**.

The requested path survey found no `/opt/*isaac*` path and no
`~/workspaces/isaac_ros*` path. Isaac-named paths directly under this user's home were:

```text
/home/aisc216/isaac_env
/home/aisc216/isaac_ros_venv
/home/aisc216/isaac_ros_ws
/home/aisc216/isaacsim
/home/aisc216/isaacsim_assets
/home/aisc216/isaacsim_curobo
/home/aisc216/isaacsimlab
```

A case-insensitive read-only check also found:

```text
/home/aisc216/Isaac+ROS
/home/aisc216/IsaacLab
/home/aisc216/IsaacSim-ros_workspaces
```

Their presence does not establish that an Isaac ROS workspace is built, sourced, or usable.

## 5. cuRobo imported by this project

`scripts/run_curobo_v2_viser.sh` selects:

- Python environment: `/home/aisc216/curobo_ros_env`
- cuRobo source: `/home/aisc216/ur5_ws/src/ur5_moveit_demo/curobo`
- CUDA toolkit environment: `/usr/local/cuda-12.8`

Using the same setup and `PYTHONPATH` as that script, Python reported:

```text
python=/home/aisc216/curobo_ros_env/bin/python
curobo_file=/home/aisc216/ur5_ws/src/ur5_moveit_demo/curobo/curobo/__init__.py
curobo___version__=0.8.0.post1.dev43
distribution_version=not found
```

The source checkout was at commit `78fd485fa82d9b9a063fb4985e371814587e666a`, described
by Git as `v0.8.0-43-g78fd485`. The absence of Python distribution metadata is consistent
with importing directly from the source path; the package's own `__version__` is the usable
version fact.

## 6. Official NVIDIA compatibility facts

These facts were checked against NVIDIA's versioned documentation on 2026-11-05:

- **ROS 2 Jazzy release:** Isaac ROS **4.6** explicitly says all Isaac ROS packages are
  designed and tested for ROS 2 Jazzy. Its apt instructions use the `release-4.6` repository.
  Isaac ROS 5.0 instead states compatibility with ROS 2 Lyrical, so the applicable supported
  line for this Jazzy host is release 4.6/release-4, not release 5.0.
  - <https://nvidia-isaac-ros.github.io/v/release-4.6/getting_started/index.html>
  - <https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html>
- **Isaac ROS camera families generally:** the 4.6 sensor setup page officially lists
  RealSense, ZED, Leopard Imaging Eagle VB1940 (CoE), and Leopard Imaging Hawk (GMSL).
  - <https://nvidia-isaac-ros.github.io/v/release-4.6/getting_started/sensors/index.html>
- **cuMotion with the current manipulation reference workflow:** the workflow configuration
  accepts `REALSENSE` or `ISAAC_SIM` as `camera_type`, and NVIDIA's application notes say
  the real-hardware workflows were tested on Jetson AGX Thor with one or two RealSense D455
  cameras. This is narrower than the general Isaac ROS camera-family list.
  - <https://nvidia-isaac-ros.github.io/v/release-4.6/reference_workflows/isaac_for_manipulation/packages/isaac_ros_manipulation_bringup/manipulation_workflow_config_guide.html>
  - <https://nvidia-isaac-ros.github.io/v/release-4.6/reference_workflows/isaac_for_manipulation/index.html>
- **nvblox:** the 4.6 supported-configuration table covers RealSense and ZED cameras,
  Isaac Sim camera input, and Ouster OS1 LiDAR rosbag input. On `x86+GPU` with a live sensor,
  the table lists one RealSense or one ZED camera at 30 Hz; feature coverage differs by
  sensor and mode.
  - <https://nvidia-isaac-ros.github.io/v/release-4.6/concepts/scene_reconstruction/nvblox/index.html>

## Interpretation

The local name "cuRobo" currently maps unambiguously to a standalone cuRobo source checkout
used by this repository's Python environment. The survey does **not** find evidence that the
Isaac ROS cuMotion package or the full nvblox runtime is installed in the normal Jazzy ROS
environment. The installed `realsense2_camera*` packages, RealSense vendor tools, and
`nvblox_msgs` are compatible with the senior's hypothesis, but they do not prove that
cuMotion/nvblox or a camera-connected pipeline is present.

## Open questions for the senior

1. Which physical camera model and serial number belongs to this lab setup, and was it simply
   unplugged during the survey?
2. Does "cuRobo" mean the standalone source checkout used by `run_curobo_v2_viser.sh`, or an
   Isaac ROS 4.6 cuMotion + nvblox deployment?
3. Is the intended Isaac ROS installation in `/home/aisc216/isaac_ros_ws`, another listed
   Isaac directory, or a Docker image that this user currently cannot inspect?
4. If Docker is the intended runtime, should this user be granted non-sudo access to the
   Docker daemon, or should a senior provide the relevant image/container inventory?
5. Is the target camera a RealSense D455 (the NVIDIA-validated cuMotion reference-workflow
   camera), and is the intended deployment this x86 RTX 3080 host or a Jetson system?

## Follow-up — existing Isaac folders and camera

Follow-up date: 2026-11-05 (Asia/Taipei)

This was a read-only inspection. No build, launch, install, or modification was performed in
the surveyed Isaac folders. The account shown as filesystem owner is not evidence of who
created a folder; creator identity remains unknown.

### Folder inventory

| Path | Filesystem owner | Directory last-modified date |
| --- | --- | --- |
| `/home/aisc216/isaac_ros_ws` | `aisc216:nogroup` | `2026-05-28 17:45:57 +0800` |
| `/home/aisc216/Isaac+ROS` | `aisc216:aisc216` | `2026-03-10 06:26:44 +0800` |
| `/home/aisc216/isaacsim_curobo` | `aisc216:aisc216` | `2026-05-28 17:18:28 +0800` |
| `/home/aisc216/isaac_ros_venv` | `aisc216:aisc216` | `2026-05-15 06:57:51 +0800` |

Top-level contents of `/home/aisc216/isaac_ros_ws`:

```text
build/
install/
log/
src/
cuda_12.1.1_530.30.02_linux.run
cuda-keyring_1.1-1_all.deb
cuda-keyring_1.1-1_all.deb.1
kick_cuda.py
```

Top-level contents of `/home/aisc216/Isaac+ROS`:

```text
direct_isaac_command_test.py
initial_state_move_S2R_switch.py
isaac_ros2control_adapter_jointstate_cmd.py
ur5_with_table_1008.usd
```

Top-level contents of `/home/aisc216/isaacsim_curobo`:

```text
.vscode/                 extsDeprecated/          requirements.txt
LICENSE.txt              extsUser/                run_tests.py
PACKAGE-LICENSES/        extscache/               setup_conda_env.sh
VERSION                  extension_examples ->   setup_python_env.sh
apps/                    isaac-sim.action_and_event_data_generation.sh
clear_caches.sh          isaac-sim.compatibility_check.sh
config/                  isaac-sim.fabric.sh      setup_ros_env.sh
data/                    isaac-sim.selector.sh    standalone_examples/
docs/                    isaac-sim.sh             tensorboard
environment.yml          isaac-sim.streaming.sh   tests/
exts/                    isaac-sim.xr.vr.sh       tools/
isaacsim/                jupyter_kernel/          warmup.sh
jupyter_notebook.sh      kit/
post_install.sh          python.sh
python_packages/
```

The `VERSION` file contains `5.1.0-rc.19+release.26219.9c81211b.gl`.

Top-level contents of `/home/aisc216/isaac_ros_venv`:

```text
bin/
include/
lib/
lib64 -> lib/
share/
pyvenv.cfg
```

### `/home/aisc216/isaac_ros_ws` details

`src/`, `build/`, and `install/` all exist. The top level of `src/` contains:

```text
isaac_ros_common/
isaac_ros_cumotion/
log/
```

Declared ROS packages found under `src/` are:

- `isaac_ros_common` repository, package version `4.4.0`: `isaac_common`,
  `isaac_common_py`, `isaac_ros_apriltag_interfaces`, `isaac_ros_common`,
  `isaac_ros_launch_utils`, `isaac_ros_nitros_bridge_interfaces`,
  `isaac_ros_nova_interfaces`, `isaac_ros_pointcloud_interfaces`,
  `isaac_ros_r2b_galileo`, `isaac_ros_tensor_list_interfaces`, `isaac_ros_test`, and
  `isaac_ros_test_cmake`.
- `isaac_ros_cumotion` repository, package version `4.2.0`: `curobo_core`,
  `isaac_ros_cumotion`, `isaac_ros_cumotion_examples`,
  `isaac_ros_cumotion_interfaces`, `isaac_ros_cumotion_moveit`,
  `isaac_ros_cumotion_object_attachment`, `isaac_ros_cumotion_python_utils`,
  `isaac_ros_cumotion_robot_description`, `isaac_ros_esdf_visualizer`,
  `isaac_ros_goal_setter_interfaces`, and `isaac_ros_moveit_goal_setter`.

Git identities:

- `src/isaac_ros_common`: remote
  `https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git`, branch `main`, HEAD
  `d7164701715edfd4adbec87355461069bc8071d6`, tag `v4.4-0` at HEAD. Its working tree
  already had a modification to `isaac_ros_common/CMakeLists.txt`; this survey did not alter it.
- `src/isaac_ros_cumotion`: remote
  `https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git`, branch `release-4.2`, HEAD
  `b4f12671447afbc8db12017d0f79bebebf941781`, no tag at HEAD.
- Nested `src/isaac_ros_cumotion/curobo_core/curobo`: remote
  `https://github.com/NVlabs/curobo`, detached HEAD
  `0a50de1ba72db304195d59d9d0b1ed269696047f`, tag `v0.7.7` at HEAD.

cuMotion is present in both source and install space. The installed top-level package
directories are:

```text
isaac_ros_cumotion
isaac_ros_cumotion_examples
isaac_ros_cumotion_interfaces
isaac_ros_cumotion_moveit
isaac_ros_cumotion_object_attachment
isaac_ros_cumotion_python_utils
isaac_ros_cumotion_robot_description
```

An nvblox ROS package is **not found** in either `src/` or `install/`. The cuRobo source tree
does contain benchmark, example, and test filenames containing `nvblox`, but those files do
not constitute an installed nvblox ROS package.

### Camera follow-up

The camera is **still not connected**:

- `lsusb` showed the same hubs, mouse, Bluetooth adapter, MSI lighting device, keyboard, and
  USB hub as the first survey, with no camera device.
- `/dev/video*`: **not found**.
- `rs-enumerate-devices -s`: `No device detected. Is it plugged in?`

No video was streamed and `realsense2_camera` was not started.
