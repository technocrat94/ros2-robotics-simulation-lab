# Newton robot-link depth camera check

Check date: 2026-11-06 (Asia/Taipei)

## Verdict

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
