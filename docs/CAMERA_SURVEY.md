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
