# D435 and cuMotion start check

Check date: 2026-11-05 (Asia/Taipei)

Branch: `experiment/d435-cumotion`

This check used no real robot, UR driver, robot IP connection, package installation, build,
or modification of `/home/aisc216/isaac_ros_ws`. The camera was used only for live ROS topic
rate checks; no video or screenshots were recorded.

## Existing working-tree state left untouched

The following state existed before this task and was deliberately not staged, restored,
removed, or otherwise changed:

```text
 D docs/experiments/rigid-obstacles/media/05_c2_tall_wall_refused.png
?? curobo/
```

The untracked `curobo/` directory is a Git checkout with:

```text
remote:  https://github.com/NVlabs/curobo.git
branch:  main
commit:  78fd485fa82d9b9a063fb4985e371814587e666a
describe: v0.8.0-43-g78fd485
```

This is the source checkout used by `scripts/run_curobo_v2_viser.sh`; it was not added to the
outer repository.

## Summary

- The connected Intel RealSense D435 passed the USB safety gate: USB type `3.2`.
- Depth, color, and pointcloud streams all published in ROS domain 20. The observed rates with
  three concurrent `ros2 topic hz` subscribers were approximately 23.48, 16.63, and 11.20 Hz.
- The installed cuMotion packages resolve and were built against ROS 2 Jazzy.
- Matching UR5e+Robotiq descriptions are present, but the planner did not reach warmup or ready.
  Both launch processes exited while importing the workspace's cuRobo 0.7.7 because the sourced
  Python environment lacks `setuptools_scm`.
- No plan request was sent because the planner never became ready. No process started by this
  check was left running.

## GPU pre-check

Command:

```bash
nvidia-smi
```

Result before testing:

```text
GPU: NVIDIA GeForce RTX 3080
Driver: 580.178.04
CUDA reported by driver: 13.0
Memory: 868 MiB / 10240 MiB
GPU utilization: 12%
```

Only existing desktop processes were listed. No other user's process was stopped.

## Part 0: is `/home/aisc216/isaac_ros_ws` in active use?

This check was performed read-only. Neither repository has a local `git config user.name` value.

Recent repository history:

```text
isaac_ros_common:
  Jaiveer Singh | 2026-04-30T19:58:46-07:00 | Merge pull request #236 from NVIDIA-ISAAC-ROS/release-4.4
  Kajanan Chinniah | 2026-04-30T19:52:12-07:00 | Isaac ROS 4.4
  chengronglai | 2026-03-23T19:29:00-07:00 | Merge pull request #233 from NVIDIA-ISAAC-ROS/release-4.3

isaac_ros_cumotion:
  Kajanan Chinniah | 2026-02-19T20:32:18-08:00 | Isaac ROS 4.2
  kajananchinniahNV | 2026-02-02T20:58:52-08:00 | Merge pull request #48 from NVIDIA-ISAAC-ROS/release-4.1
  Kajanan Chinniah | 2026-02-02T19:12:02-08:00 | Isaac ROS 4.1
```

Newest observed files:

| Area | Newest file and timestamp |
| --- | --- |
| `build/` | `compile_commands.json` — `2026-05-28 17:49:42 +0800` |
| `install/` | `isaac_ros_cumotion/.../launch/__pycache__/isaac_ros_cumotion.launch.cpython-312.pyc` — `2026-11-05 17:20:59 +0800` |
| `log/` | `build_2026-05-28_17-45-57/logger_all.log` — `2026-05-28 17:49:42 +0800` |

The install-space Python cache timestamp coincides with this task's planner launch. It was an
automatic Python import cache produced while running the existing launch file, not a build or
manual source edit; it was left unchanged after observation.

Commands used for active-use checks:

```bash
ps -eo user,args | grep isaac_ros_ws
who
grep -l isaac_ros_ws ~/.bashrc ~/*.sh
```

Results:

- No host process command referenced `isaac_ros_ws` after excluding the inspection command
  itself.
- `who` showed only `aisc216` on `seat0` and display `:1`; no other login was listed.
- `/home/aisc216/use_cumotion_rviz_env.sh` was the only matched startup/script file.
  `/home/aisc216/.bashrc` did not match.
- No shell history or other user's home directory was read.

No active workspace user was found at check time. The start test described below had already
finished and left no process running.

## Part 1: Intel RealSense D435

### Device identity and USB mode

Commands:

```bash
lsusb
ls -l /dev/video*
rs-enumerate-devices -s
rs-enumerate-devices | rg -i \
  'Device Name|Serial Number|Firmware Version|USB Type Descriptor|Product Id|Physical Port|Advanced Mode'
```

Observed facts:

| Field | Value |
| --- | --- |
| Model | Intel RealSense D435 |
| USB ID | `8086:0b07` |
| Serial number | `346522070779` |
| ASIC serial number | `345423023386` |
| Firmware | `5.17.0.10` (also the recommended version) |
| Product ID | `0B07` |
| USB Type Descriptor | `3.2` |
| Advanced mode | `YES` |
| Linux video devices | `/dev/video0` through `/dev/video5` |

USB type is 3.x, so the task continued. The physical sysfs path reported by librealsense was:

```text
/sys/devices/pci0000:00/0000:00:14.0/usb2/2-7/2-7:1.0/video4linux/video0
```

The `usb2` controller path component does not override the device's negotiated USB Type
Descriptor; both librealsense and the ROS node reported `3.2`.

### ROS camera start and measurements

The command that worked was:

```bash
mkdir -p /tmp/d435-cumotion-ros-log /tmp/d435-cumotion-ros-home
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=20
export ROS_LOG_DIR=/tmp/d435-cumotion-ros-log
export ROS_HOME=/tmp/d435-cumotion-ros-home
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  enable_color:=true \
  pointcloud.enable:=true
```

The node reported:

```text
RealSense ROS v4.56.4
Built with LibRealSense v2.56.4
Running with LibRealSense v2.56.4
Device USB type: 3.2
Depth profile: 848x480x30, Z16
Color profile: 640x480x30, RGB8
RealSense Node Is Up!
```

The topics tested were:

```text
/camera/camera/depth/image_rect_raw
/camera/camera/color/image_raw
/camera/camera/depth/color/points
```

Each rate command used the same sourced environment and ROS domain:

```bash
timeout 10s ros2 topic hz /camera/camera/depth/image_rect_raw
timeout 10s ros2 topic hz /camera/camera/color/image_raw
timeout 10s ros2 topic hz /camera/camera/depth/color/points
```

They ran concurrently so that all streams were measured over the same interval. The last
reported averages were:

| Stream | Observed average | Configured sensor profile |
| --- | ---: | ---: |
| Depth | `23.479 Hz` | `30 Hz` |
| Color | `16.627 Hz` | `30 Hz` |
| Pointcloud | `11.196 Hz` | generated from live streams |

The lower observed subscriber rates were measured under the load of three concurrent Python
CLI subscribers and pointcloud generation. They are observations, not a claim that the sensor
profiles changed from the 30 Hz values logged by the driver.

Frame IDs were obtained with:

```bash
ros2 topic echo --once --field header.frame_id /camera/camera/depth/image_rect_raw
ros2 topic echo --once --field header.frame_id /camera/camera/color/image_raw
ros2 topic echo --once --field header.frame_id /camera/camera/depth/color/points
```

| Topic | `frame_id` |
| --- | --- |
| Depth image | `camera_depth_optical_frame` |
| Color image | `camera_color_optical_frame` |
| Pointcloud | `camera_depth_optical_frame` |

One depth message-loss notice appeared while the one-shot CLI subscriber joined; it still
received the next message and returned the frame ID.

The launch was stopped with Ctrl-C. Both Depth Module and RGB Camera logged clean shutdown,
and the camera could immediately be enumerated again.

### How to view the camera later

Only one program should open the D435 at a time.

For the native viewer, first stop any `realsense2_camera` process, then run:

```bash
realsense-viewer
```

For RViz pointcloud viewing, close `realsense-viewer`, then use two terminals.

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  enable_color:=true \
  pointcloud.enable:=true
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=20
rviz2
```

In RViz, set **Fixed Frame** to `camera_depth_optical_frame`, select **Add → By topic**, and
add `/camera/camera/depth/color/points` as `PointCloud2`. Stop the ROS camera launch before
opening `realsense-viewer` again.

## Part 2: cuMotion start test

### Package resolution and build ROS distribution

Commands:

```bash
set +u
source /opt/ros/jazzy/setup.bash
source /home/aisc216/isaac_ros_ws/install/setup.bash
set -u
echo "$ROS_DISTRO"
ros2 pkg prefix isaac_ros_cumotion
ros2 pkg prefix isaac_ros_cumotion_examples
ros2 pkg prefix isaac_ros_cumotion_interfaces
ros2 pkg prefix isaac_ros_cumotion_moveit
ros2 pkg prefix isaac_ros_cumotion_object_attachment
ros2 pkg prefix isaac_ros_cumotion_robot_description
```

All six packages resolved below `/home/aisc216/isaac_ros_ws/install/`, and `ROS_DISTRO` was
`jazzy`. The build record confirms this independently:

```text
/home/aisc216/isaac_ros_ws/install/setup.bash:
  COLCON_CURRENT_PREFIX="/opt/ros/jazzy"

/home/aisc216/isaac_ros_ws/build/isaac_ros_cumotion_interfaces/
  colcon_command_prefix_build.sh.env:
  ROS_DISTRO=jazzy
```

### UR5e robot description selected

The installed robot-description package provides:

```text
urdf/ur5e_robotiq_2f_85.urdf
urdf/ur5e_robotiq_2f_140.urdf
xrdf/ur5e.xrdf
xrdf/ur5e_robotiq_2f_85.xrdf
xrdf/ur5e_robotiq_2f_140.xrdf
```

The start test selected the matching pair:

```text
URDF: ur5e_robotiq_2f_85.urdf
XRDF: ur5e_robotiq_2f_85.xrdf
```

The XRDF uses `base_link`, the six UR arm joints, `grasp_frame` as its tool frame, collision
spheres for the UR5e and Robotiq 2F-85, and an `attached_object` fixed frame. The URDF contains
a real `ur_robot_driver/URPositionHardwareInterface` configuration and an embedded robot IP,
so it was **not** launched as a driver. It was supplied only as a file path to the standalone
cuMotion planner initialization. No UR launch file, controller, fake controller, or real
hardware process was started.

### Planner start result

The exact start command was:

```bash
set +u
source /opt/ros/jazzy/setup.bash
source /home/aisc216/isaac_ros_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=20
export ROS_LOG_DIR=/tmp/d435-cumotion-ros-log
export ROS_HOME=/tmp/d435-cumotion-ros-home
ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
  cumotion_planner.urdf_path:=/home/aisc216/isaac_ros_ws/install/isaac_ros_cumotion_robot_description/share/isaac_ros_cumotion_robot_description/urdf/ur5e_robotiq_2f_85.urdf \
  cumotion_planner.yml_file_path:=/home/aisc216/isaac_ros_ws/install/isaac_ros_cumotion_robot_description/share/isaac_ros_cumotion_robot_description/xrdf/ur5e_robotiq_2f_85.xrdf \
  cumotion_planner.tool_frame:=grasp_frame \
  cumotion_planner.read_esdf_world:=false
```

Both `static_planning_scene` and `cumotion_goal_set_planner_node` started as processes, then
immediately exited with code 1. The decisive error from both was:

```text
File "/home/aisc216/isaac_ros_ws/build/curobo_core/curobo/src/curobo/__init__.py", line 55, in _get_version
  import setuptools_scm
ModuleNotFoundError: No module named 'setuptools_scm'
```

Result:

- Planner process started: **yes, then exited during Python import**.
- Reached `warming up cuMotion`: **no**.
- Reached `cuMotion is ready for planning queries!`: **no**.
- Used the GPU: **no evidence; failure occurred before CUDA/cuMotion initialization**.
- Plan-only request: **not sent**, because no action server became ready.
- Planning success/time: **not applicable**.

Best cause assessment: this is first a missing Python runtime dependency in the environment
produced by sourcing Jazzy and the existing workspace. The import path is the workspace's
cuRobo 0.7.7 checkout. The separate project checkout identifies as cuRobo 0.8.0, but this run
failed before exercising either planner API, so the observed error does not establish a
0.7.7-versus-0.8.0 compatibility failure. Per the task rules, no dependency was installed and
no environment or workspace file was changed.

After the failure, a process check found no `realsense2_camera`,
`cumotion_goal_set_planner_node`, or `static_planning_scene` process left running.

## Retry — cuMotion dedicated environment

Retry date: 2026-11-05 (Asia/Taipei)

This retry followed `/home/aisc216/use_cumotion_rviz_env.sh` without editing it or the
workspace.

### Environment helper

`use_cumotion_rviz_env.sh` does the following:

1. Sources `/home/aisc216/use_curobo_ros_env.sh`, which sets CUDA 12.8 through
   `CUDA_HOME=/usr/local/cuda-12.8`, prepends its `bin` and `lib64` paths, sources ROS 2
   Jazzy and `/home/aisc216/ur5_policy_no_gripper_state/install/setup.bash`, then activates
   `/home/aisc216/curobo_ros_env`.
2. Sources `/home/aisc216/isaac_ros_ws/install/setup.bash`.
3. Prepends the cuMotion workspace's bundled cuRobo source and the venv's Python 3.12
   site-packages to `PYTHONPATH`, in that order:

```text
CUROBO_CORE_PATH=/home/aisc216/isaac_ros_ws/src/isaac_ros_cumotion/curobo_core/curobo/src
CUROBO_ROS_VENV_SITE=/home/aisc216/curobo_ros_env/lib/python3.12/site-packages
```

The ordering intentionally selects cuMotion's bundled cuRobo instead of the separate
`/home/aisc216/ur5_ws/src/ur5_moveit_demo/curobo` checkout while still exposing venv
dependencies to ROS console scripts.

### Import check

Command:

```bash
set +u
source /home/aisc216/use_cumotion_rviz_env.sh
set -u
export PYTHONDONTWRITEBYTECODE=1
python - <<'PY'
import sys
import setuptools_scm
import curobo
from importlib.metadata import version
print(sys.executable)
print(setuptools_scm.__file__)
print(version('setuptools-scm'))
print(curobo.__file__)
print(curobo.__version__)
PY
```

Result:

```text
Python: /home/aisc216/curobo_ros_env/bin/python
setuptools_scm: /home/aisc216/curobo_ros_env/lib/python3.12/site-packages/setuptools_scm/__init__.py
setuptools-scm distribution: 10.0.5
cuRobo: /home/aisc216/isaac_ros_ws/src/isaac_ros_cumotion/curobo_core/curobo/src/curobo/__init__.py
cuRobo reported version: 0.0.post1.dev57
ROS_DISTRO: jazzy
CUDA_HOME: /usr/local/cuda-12.8
```

The dedicated environment therefore fixed the first attempt's missing `setuptools_scm`
dependency and selected the intended workspace cuRobo source.

### Planner retry

The exact retry command was:

```bash
set +u
source /home/aisc216/use_cumotion_rviz_env.sh
set -u
export ROS_DOMAIN_ID=20
export ROS_LOG_DIR=/tmp/d435-cumotion-ros-log
export ROS_HOME=/tmp/d435-cumotion-ros-home
export PYTHONDONTWRITEBYTECODE=1
ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
  cumotion_planner.urdf_path:=/home/aisc216/isaac_ros_ws/install/isaac_ros_cumotion_robot_description/share/isaac_ros_cumotion_robot_description/urdf/ur5e_robotiq_2f_85.urdf \
  cumotion_planner.yml_file_path:=/home/aisc216/isaac_ros_ws/install/isaac_ros_cumotion_robot_description/share/isaac_ros_cumotion_robot_description/xrdf/ur5e_robotiq_2f_85.xrdf \
  cumotion_planner.tool_frame:=grasp_frame \
  cumotion_planner.read_esdf_world:=false
```

Both `static_planning_scene` and `cumotion_goal_set_planner_node` started. The missing Python
dependency did not recur, but the existing CUDA extensions could not load. The decisive error
was:

```text
ImportError: /home/aisc216/isaac_ros_ws/src/isaac_ros_cumotion/curobo_core/curobo/src/curobo/curobolib/geom_cu.cpython-312-x86_64-linux-gnu.so: undefined symbol: _ZN3c106detail23torchInternalAssertFailEPKcS2_jS2_RKSs
```

The fallback then announced prohibited build work:

```text
kinematics_fused_cu not found, JIT compiling...
geom_cu binary not found, jit compiling...
```

PyTorch also warned that `TORCH_CUDA_ARCH_LIST` was unset. The launch was interrupted with
Ctrl-C immediately after the JIT requirement was identified, because the task explicitly
forbids building or rebuilding the workspace. The resulting `KeyboardInterrupt` traces came
from stopping the two compile/import paths; no further fix was attempted.

Result:

- Reached `cuMotion is ready for planning queries!`: **no**.
- Planner action server ready: **no**.
- Plan-only goal: **not sent**.
- Planning result/time: **not applicable**.
- Planner GPU memory: **not established**; failure occurred in extension import/JIT before
  planner warmup. After shutdown, total GPU usage was `858 MiB / 10240 MiB`, and the only
  compute process reported by `nvidia-smi` was the pre-existing GNOME remote desktop daemon
  using `259 MiB`.

Best cause assessment: the workspace's existing cuRobo CUDA extension binary was built against
a different PyTorch/C10 ABI than the PyTorch currently supplied by `curobo_ros_env`. The missing
or incompatible binary causes cuRobo to fall back to JIT compilation. A compatible rebuild would
be required to proceed, but that is outside this task's allowed actions. This is distinct from
the first attempt's missing `setuptools_scm` and does not implicate the project's separate cuRobo
0.8.0 checkout, because this retry correctly loaded the workspace-bundled source.

No retry process remained afterward. A read-only timestamp check showed that the existing
workspace CUDA `.so` files still had their original 2026-05-28 modification times; the retry
did not replace them. `PYTHONDONTWRITEBYTECODE=1` prevented new Python bytecode cache files.
