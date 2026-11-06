# Newton robot-link depth camera check

Check date: 2026-11-06 (Asia/Taipei)

## Verdict

> **Correction (2026-11-06):** The benchmark in this first section rendered
> no box, sphere, or cylinder. Its `0.2277` / `0.2610 ms/frame` timings and
> depth summaries describe robot geometry plus ground only, not the stated
> target-object scene. The numbers below are retained unchanged as historical
> measurements and must not be quoted as performance or depth for that scene.

**Yes.** The already-installed Newton 1.5.1 `SensorTiledCamera` rendered a
640 x 480 depth image on the RTX 3080 from a camera site rigidly attached to
the UR5 `tool0` link. No package was installed or upgraded.

The steady-state median was **0.2277 ms/frame** for link-camera transform plus
depth rendering, and **0.2610 ms/frame** when Newton forward kinematics and
shape-BVH refit were also run every frame. These are GPU timings without a
host image download, ROS message construction, or display.

## Existing environment

| Item | Measured value |
| --- | --- |
| Newton | 1.5.1 |
| Warp | 1.17.0 |
| Device | `cuda:0`, NVIDIA GeForce RTX 3080, sm_86 |
| GPU memory | 10,240 MiB |
| NVIDIA driver | 580.178.04 |
| Warp CUDA toolkit | 12.9 |
| Robot asset | Existing `~/newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf` |
| Image | 640 x 480, float32 ray-hit depth in metres |
| Vertical FOV | 58 degrees |

The Python interpreter was the existing
`~/newton_ws/.venv-cpu/bin/python`. Despite the environment's historical
`-cpu` name, Warp enumerated both `cpu` and the RTX 3080, and this check
explicitly used `cuda:0`.

## Method

The reproducible check is
[`scripts/newton_link_depth_camera_benchmark.py`](../scripts/newton_link_depth_camera_benchmark.py).
It uses Newton's public `newton.sensors.SensorTiledCamera` API and does the
following:

1. Imports the existing UR5e/Robotiq URDF into a Newton model.
2. Adds a Newton `SITE` named `ur/tool0_depth_camera` to body 11,
   `ur/tool0`, with a fixed local translation of `[0.18, 0.0, -0.08]` m and a
   180-degree local-X rotation. The side offset prevents the gripper from
   filling the image; the optical `-Z` axis looks down in the baseline pose.
3. Adds three simple targets and a ground plane, then renders the public
   `depth_image` output. This channel is ray-hit distance, not forward-axis Z
   depth; Newton 1.5.1 also exposes a separate `forward_depth_image` channel.
4. Composes the site's fixed local transform with the current `tool0` body
   transform on the GPU before every render.
5. Changes `shoulder_lift_joint` by 0.25 rad, recomputes FK and the shape BVH,
   and verifies that both the camera world pose and depth image change.
6. Warms up 20 frames, then measures five repeats of 120 frames with one GPU
   synchronization at the end of each repeat.

Exact command:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
~/newton_ws/.venv-cpu/bin/python \
  scripts/newton_link_depth_camera_benchmark.py \
  --urdf ~/newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf \
  --device cuda:0 \
  --width 640 --height 480 \
  --warmup 20 --frames 120 --repeats 5
```

No `pip`, `apt`, bootstrap, or other install command was run.

## Attachment and depth validation

The finalized model identified the camera as a site shape and retained the
requested parent:

| Check | Result |
| --- | --- |
| Camera parent | body 11, `ur/tool0` |
| Camera site | shape 54, `SITE=true` |
| Camera translation change after joint motion | 0.160777 m |
| Pixels changing by more than 0.1 mm | 98.0915% |
| Valid depth, pose A | 307,200 / 307,200 pixels |
| Valid depth, pose B | 307,200 / 307,200 pixels |

Depth summaries:

| Pose | Minimum | Median | Maximum |
| --- | ---: | ---: | ---: |
| Baseline arm pose | 0.183064 m | 0.572455 m | 0.696300 m |
| Shoulder-lift +0.25 rad | 0.183064 m | 0.432441 m | 0.618413 m |

Camera-to-world transforms are stored as translation followed by quaternion
`[x, y, z, qx, qy, qz, qw]`:

```text
pose A: [0.4869000, -0.0708499, 0.5118590,
         0.0000001, -0.0000001, -0.7071067, 0.7071068]
pose B: [0.5763412, -0.0708500, 0.3782573,
        -0.0881583, 0.0881583, -0.7015897, 0.7015898]
```

This confirms more than a fixed world camera: the camera pose is derived from
the robot link on every frame, and changing an upstream joint changes the
rendered image.

## Performance

The benchmark uses `time.perf_counter()` around each 120-frame batch and calls
`wp.synchronize_device(cuda:0)` before recording the batch end. The reported
number therefore includes Python dispatch and completed GPU work, but not
copying the final 1.2 MiB float32 image to host memory.

### Link transform plus depth render

This loop updates the link-attached camera transform and calls
`SensorTiledCamera.update(..., depth_image=...)`.

| Repeat | ms/frame |
| ---: | ---: |
| 1 | 0.226494 |
| 2 | 0.229296 |
| 3 | 0.228634 |
| 4 | 0.227689 |
| 5 | 0.226966 |
| **Median** | **0.227689** |
| Range | 0.226494-0.229296 |
| Median equivalent rate | 4,391.9 frames/s |

### Robot-frame pipeline plus depth render

This loop additionally runs `newton.eval_fk(...)` and
`model.bvh_refit_shapes(...)` before the camera transform and depth render.

| Repeat | ms/frame |
| ---: | ---: |
| 1 | 0.287096 |
| 2 | 0.286909 |
| 3 | 0.259771 |
| 4 | 0.261025 |
| 5 | 0.260490 |
| **Median** | **0.261025** |
| Range | 0.259771-0.287096 |
| Median equivalent rate | 3,831.0 frames/s |

The first rendered frame in the final, cached benchmark process took
40.629 ms. On the first attempt before the camera kernels existed in Warp's
cache, the console separately reported approximately 629 ms to compile camera
rays, 59 ms for the link-camera kernel, and 1,362 ms for the render
megakernel. These one-time JIT costs are excluded from the steady-state
figures; no cache was deleted or manually changed.

## Scope and limitations

- The test proves Newton 1.5.1 can render depth from a transform rigidly tied
  to a robot link; it does not add a physical camera body or collision shape.
- The scene contains one robot, one camera, three primitive targets, and a
  ground plane. More worlds, cameras, meshes, particles, shadows, or color
  channels will change performance.
- The render-only number keeps the robot pose static after warmup. The fuller
  pipeline repeats FK and BVH refit each frame, but does not run a physics
  solver or alter the joint value during the timed loop.
- The timing excludes GPU-to-CPU image transfer, ROS 2 publication, camera
  noise, clipping/post-processing, and visualization. It must not be quoted
  as an end-to-end ROS camera frame time.
- The initial failed validation was caused by placing the camera directly
  behind the gripper, which made the gripper fill the view. Moving the fixed
  mount 18 cm to the side produced a useful scene view; no Newton source or
  environment package was modified.

## Point cloud and Viser

Check date: 2026-11-06 (Asia/Taipei)

### Verdict

The existing Newton 1.5.1 and Viser 1.0.26 environments can turn the simulated
link-mounted depth image into a world-frame point cloud and display it over the
robot and analytic scene without installing anything. The reproducible script
is [`scripts/newton_sim_depth_cloud_viser.py`](../scripts/newton_sim_depth_cloud_viser.py).

This test uses 848 x 480 `forward_depth_image`, not `depth_image`. Newton's
`camera_fovs` argument is the **vertical field of view in radians**. The
Newton pinhole kernel uses `tan(vertical_fov / 2)` and multiplies by
`width / height` for the horizontal extent. Therefore 58 degrees vertical at
848 x 480 corresponds to 88.8004 degrees horizontal.

The `[0.18, 0.0, -0.08]` m `tool0` mount remains a **placeholder for this
simulation check; it is not the real camera mount**.

### Scene correction needed for this check

The previous benchmark created the box, sphere, and cylinder as unjointed
bodies and then assigned `q[6:] = 0.0`. A direct A/B reproduction proved that
this blanket assignment, rather than `eval_fk` by itself, was the cause: with
the model's initialized suffix preserved, the three body positions matched
their requested positions within `1.19e-8 m`; with the suffix zeroed, all three
became `[0, 0, 0]`, with at least `0.3805 m` position error. The earlier camera
therefore rendered only robot geometry and ground.

This script retains the exact previous dimensions, centers, colors, and ground,
but creates the three targets as world-static Newton shapes (`body=-1`). That
makes both the rendered truth and the analytic surface check correspond to the
documented scene. A moving obstacle instead needs its own seven free-joint
coordinates (XYZ plus a normalized XYZW quaternion) maintained or written each
frame, followed by FK and shape-BVH refit; it must not zero the entire joint
suffix.

| Surface | Center (m) | Size |
| --- | --- | --- |
| Box | `[0.487, 0.109, 0.060]` | half extents `[0.120, 0.100, 0.060]` m |
| Sphere | `[0.680, 0.050, 0.080]` | radius `0.080` m |
| Cylinder | `[0.320, 0.180, 0.100]` | radius `0.055` m, half-height `0.100` m, Z axis |
| Ground | z = `0` | infinite analytic plane |

### Same-frame back-projection and robot pixels

For each capture, the script performs FK, refits the shape BVH, derives the
camera pose from the `tool0` site, and renders forward depth, ray depth, and
shape index in one `SensorTiledCamera.update`. It then synchronizes once and
copies all four values without changing state. The depth and camera transform
therefore cannot come from different frames.

For pixel center `(px + 0.5, py + 0.5)`, forward depth `d`, vertical half-angle
`h = tan(58 deg / 2)`, and aspect `a = 848 / 480`, the camera point is:

```text
x = d * (2 * ((px + 0.5) / 848 - 0.5) * h * a)
y = d * (-2 * ((py + 0.5) / 480 - 0.5) * h)
z = -d
```

The script then applies the camera-to-world transform from that same capture.
Newton provides `shape_index_image`, so no fallback render is needed. Robot
shapes are the original URDF shape indices 0 through 53. In every checked pose:

- Valid depth points: 407,040 / 407,040.
- Robot points: 14,929 (3.6677%), excluded from analytic scene accuracy.
- Environment points: 392,111.
- Robot points are displayed red; environment points are colored by world Z.

The `0.183064 m` minimum ray depth in the previous 640 x 480 report was not an
object or ground hit: it was the camera seeing the Robotiq left finger link.
At the new 848 x 480 sampling, the nearest ray is `0.176883 m`, at pixel
`(y=239, x=73)`, shape 34 on `ur/robotiq_85_left_finger_link`. The value changes
slightly because the two resolutions have different ray centers.

### Analytic accuracy at three arm poses

For every non-robot point, the script evaluates analytic signed distance to
the axis-aligned box, sphere, Z-axis capped cylinder, and z=0 ground plane,
then assigns the point to the surface with the smallest absolute distance.
The table reports signed median and absolute median/p95/max in millimetres.

| Pose | Surface | Count | Signed median | Abs median | Abs p95 | Abs max |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Baseline | Box | 57,900 | 0.000005 | 0.000012 | 0.000040 | 0.000064 |
| Baseline | Sphere | 16,804 | 0.000158 | 0.000158 | 0.000326 | 0.000507 |
| Baseline | Cylinder | 21,735 | -0.000012 | 0.000023 | 0.000092 | 0.000180 |
| Baseline | Ground | 295,672 | -0.000004 | 0.000022 | 0.000059 | 0.000111 |
| Shoulder lift +0.15 rad | Box | 84,133 | -0.000010 | 0.000012 | 0.000040 | 0.000062 |
| Shoulder lift +0.15 rad | Sphere | 16,115 | 0.000048 | 0.000059 | 0.000164 | 0.000299 |
| Shoulder lift +0.15 rad | Cylinder | 14,929 | 0.000034 | 0.000051 | 0.000143 | 0.000271 |
| Shoulder lift +0.15 rad | Ground | 276,934 | -0.000027 | 0.000027 | 0.000065 | 0.000111 |
| Pan +0.25, elbow -0.15 rad | Box | 42,226 | -0.000010 | 0.000018 | 0.000047 | 0.000085 |
| Pan +0.25, elbow -0.15 rad | Sphere | 13,339 | 0.000097 | 0.000130 | 0.000360 | 0.000628 |
| Pan +0.25, elbow -0.15 rad | Cylinder | 13,943 | -0.000026 | 0.000027 | 0.000057 | 0.000120 |
| Pan +0.25, elbow -0.15 rad | Ground | 322,603 | -0.000018 | 0.000026 | 0.000072 | 0.000136 |

All three poses pass the requested median `< 1 mm` and p95 `< 5 mm` target.
The sub-micrometre residuals are floating-point reconstruction error, not a
claim about real-camera accuracy.

### Negative control: ray depth incorrectly treated as forward depth

The identical pixels and camera poses were back-projected a second time while
deliberately treating Newton's ray-hit `depth_image` as forward-axis depth.
The check clearly detects the mistake:

| Pose | Abs median | Abs p95 | Abs max |
| --- | ---: | ---: | ---: |
| Baseline | 58.723 mm | 204.583 mm | 258.095 mm |
| Shoulder lift +0.15 rad | 42.238 mm | 173.900 mm | 219.013 mm |
| Pan +0.25, elbow -0.15 rad | 86.820 mm | 237.265 mm | 294.796 mm |

The median p95 rises from `0.000072 mm` with correct forward depth to
`204.583 mm` in the negative control. This test is therefore sensitive to the
depth-semantics error it is intended to catch.

### Timing

Each stage was timed separately over 120 frames using `time.perf_counter()`.
The render sample synchronizes CUDA before stopping its timer. Host-copy timing
includes forward depth, ray depth, shape index, and camera pose. Point-cloud
conversion processes all 407,040 pixels. The Viser update sends a stride-3
display cloud of 45,280 points plus robot state, frustum, depth panel, and text;
it does not include the preceding render, copy, or conversion.

| Stage | Median | p95 | Range |
| --- | ---: | ---: | ---: |
| Depth render: forward + ray + shape index | 0.4761 ms | 0.6437 ms | 0.4628-0.7614 ms |
| Copy images, shape index, and pose to host | 1.0469 ms | 1.2504 ms | 1.0033-1.3714 ms |
| Full-resolution world cloud conversion | 10.6852 ms | 11.0831 ms | 10.1955-11.4513 ms |
| Full Viser update, 45,280 displayed points | 5.9767 ms | 6.3506 ms | 5.6864-28.7526 ms |

These are component timings, not a summed streaming rate. The interactive
viewer captures only after a slider changes and otherwise waits on an event,
so it does not spin the GPU while idle.

### UR5 versus UR5e check

The loaded URDF's shoulder-link origin is `0.089159 m` above the base. This
matches UR5 (`about 0.0892 m`) rather than UR5e (`about 0.1625 m`). The asset
name contains `ur5`, and the measured kinematic offset confirms it is not a
UR5e model.

### What you should see

- The UR5 and Robotiq gripper rendered as solid Newton/Viser robot geometry.
- Solid cyan box, orange sphere, green cylinder, and gray ground truth shapes.
- A yellow camera frustum at the live placeholder `tool0` camera pose.
- The live forward-depth image both on the frustum and in the GUI panel.
- A world-frame point cloud lying on the truth surfaces; environment points
  vary from blue to yellow with height and robot hits are red.
- Six joint sliders. Moving one updates the robot, attached camera, depth, and
  cloud once; the cloud remains aligned to the world objects.
- Checkboxes for true objects, the full cloud, and robot points.
- Text readouts for current valid point count and median environment error.

The server binds only to `127.0.0.1:8083` and prints
`http://127.0.0.1:8083`. Exact one-line viewer command from any shell:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && ~/newton_ws/.venv-cpu/bin/python scripts/newton_sim_depth_cloud_viser.py
```

The noninteractive verification used the same script with
`--check-only --timing-frames 120`. It started and stopped its own Viser server;
no process was left running and no media was recorded.

## Adjustable mount

Check date: 2026-11-06 (Asia/Taipei)

The camera mount is now loaded from
[`config/sim_camera_mount.yaml`](../config/sim_camera_mount.yaml). Its current
values are deliberately labelled `unmeasured_initial_guess`: parent
`ur/tool0`, left `0.050 m`, up `0.080 m`, forward `0.030 m`, and tilt `15 deg`.
They are controls for matching the real D435 by eye, not measured hardware
dimensions.

### Model-derived directions and tilt

The directions are measured from the loaded UR5/Robotiq state, rather than
hard-coded mount axes:

- `FORWARD` is flange to the midpoint of the two fingertips. In the `tool0`
  frame it measured `[-3.96e-8, -4.82e-9, 1.0]`, confirming tool0 `+Z`.
- `LEFT` is right fingertip to left fingertip along the finger-opening axis,
  with its forward component removed. It measured `[1.0, -5.13e-8, 3.96e-8]`,
  confirming tool0 `+X` as left when looking from flange toward fingertips.
- `UP = FORWARD x LEFT`. It measured `[5.13e-8, 1.0, 4.82e-9]`, confirming
  tool0 `+Y` as up in that same view.

`forward_m` starts at the flange face. At zero tilt the optical axis is
parallel to `FORWARD`; positive `tilt_deg` rotates it toward the gripper centre
line while projecting image-up so it stays on the measured `UP` side.

### Controls

The Viser **Camera mount** folder has left/up sliders from `-0.20` to `0.20 m`,
forward from `-0.10` to `0.20 m`, and tilt from `-60` to `60 deg`. Every mount
or arm-slider change performs one new capture and updates the camera, depth
panel, world cloud, mount readout, and robot-point percentage. Labelled
`LEFT`, `UP`, and `FORWARD` arrows originate at the flange and have a visibility
checkbox. **Save mount** preserves the YAML's field order and formatting,
writes the slider values, and changes status to `matched_by_eye`.

The noninteractive check changed all four GUI handle values, observed four real
Viser slider callbacks, serviced the dirty event through the normal recapture
path, and set the button handle value to invoke its registered click callback.
It reread `matched_by_eye` from disk, observed exactly one save callback, then
restored the original YAML byte-for-byte. This tests the callbacks themselves,
not just the underlying capture and save functions.

### Accuracy at two mount settings

Both mount settings were checked at the same three arm poses. These are overall
non-robot analytic surface errors; the target remains absolute median `< 1 mm`
and p95 `< 5 mm`. The negative control deliberately treats ray distance as
forward depth.

| Mount | Arm pose | Robot points | Correct abs median | Correct abs p95 | Negative abs p95 |
| --- | --- | ---: | ---: | ---: | ---: |
| Initial `0.05, 0.08, 0.03, 15 deg` | Baseline | 6.2136% | 0.000015 mm | 0.000046 mm | 160.633 mm |
| Initial | Shoulder lift +0.15 | 6.2136% | 0.000019 mm | 0.000054 mm | 134.923 mm |
| Initial | Pan +0.25, elbow -0.15 | 6.2136% | 0.000047 mm | 0.000130 mm | 185.442 mm |
| Other `0.08, 0.06, 0.06, 25 deg` | Baseline | 12.3769% | 0.000045 mm | 0.000089 mm | 148.289 mm |
| Other | Shoulder lift +0.15 | 12.3769% | 0.000013 mm | 0.000057 mm | 121.583 mm |
| Other | Pan +0.25, elbow -0.15 | 12.3769% | 0.000067 mm | 0.000180 mm | 154.422 mm |

All six captures pass the target and both three-pose negative controls detect
the intended depth-semantics mistake. At the initial mount the nearest baseline
ray hits `ur/robotiq_85_left_inner_knuckle_link` at `0.094101 m`; at the other
mount it hits `ur/robotiq_85_left_finger_link` at `0.068403 m`.

### What the user should do

With the real robot stationary and without commanding it from this tool, open
the local viewer, use the labelled axes to compare the simulated camera with
the physical D435, adjust the four mount sliders, and click **Save mount** only
when the placement matches by eye. Exact one-line command:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && ~/newton_ws/.venv-cpu/bin/python scripts/newton_sim_depth_cloud_viser.py
```

The final verification used the same script with
`--check-only --timing-frames 120`: Newton 1.5.1 on the RTX 3080, all six
accuracy captures passing, both negative controls passing, callback checks
passing, and no install, ROS, cuRobo, real-robot command, or media recording.
The Viser server started by the check was stopped on exit.
