#!/usr/bin/env python3
"""Build and inspect the existing UR5 + Robotiq model with cuRoboV2.

This is deliberately an offline preflight.  It does not initialize ROS 2,
publish a trajectory, start Isaac Sim/Newton, or command robot hardware.
Generated URDF and collision-sphere data live below ``~/.cache`` so the
downloaded cuRobo source tree and this repository stay clean.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Any

import numpy as np
import torch
import yaml

import curobo
from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.robot_builder import RobotBuilder
from curobo.scene import Cuboid, Scene
from curobo.types import ContentPath, GoalToolPose, JointState, Pose
from curobo._src.state.state_joint_trajectory_ops import trim_joint_state_trajectory
from curobo.viewer import ViserVisualizer


ARM_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
GRIPPER_JOINT_NAME = "robotiq_85_left_knuckle_joint"
GRIPPER_OPEN_RAD = 0.0
GRIPPER_CLOSED_RAD = 0.376
TOOL_FRAME = "tool0"
APPROACH_DISTANCE_M = 0.100
LIFT_DISTANCE_M = 0.120
GRASP_CENTER_BELOW_TOOL_M = 0.159
CABLE_LENGTH_M = 0.400


def repository_root() -> Path:
    """Return the outer ur5_moveit_demo repository root."""

    return Path(__file__).resolve().parents[4]


def package_share(package_name: str) -> Path:
    """Resolve a ROS package share directory through the sourced environment."""

    result = subprocess.run(
        ["ros2", "pkg", "prefix", "--share", package_name],
        check=True,
        capture_output=True,
        text=True,
    )
    path = Path(result.stdout.strip()).resolve()
    if not path.is_dir():
        raise RuntimeError(f"ROS package share does not exist: {package_name} -> {path}")
    return path


def ensure_asset_link(asset_root: Path, package_name: str, target: Path) -> None:
    """Create the package-shaped mesh root expected by the cuRobo URDF parser."""

    link = asset_root / package_name
    if link.is_symlink() and link.resolve() == target:
        return
    if link.exists() or link.is_symlink():
        if link.is_symlink():
            link.unlink()
        else:
            raise RuntimeError(
                f"Refusing to replace non-symlink asset path: {link}. "
                "Remove it manually or choose another --cache-dir."
            )
    link.symlink_to(target, target_is_directory=True)


def generate_urdf(repo: Path, output_path: Path) -> None:
    """Expand the existing ROS Xacro into a cache-local URDF."""

    command = [
        "xacro",
        str(repo / "urdf" / "ur5_robotiq.urdf.xacro"),
        "use_fake_hardware:=true",
    ]
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    temporary_path = output_path.with_suffix(".urdf.tmp")
    temporary_path.write_text(result.stdout, encoding="utf-8")
    temporary_path.replace(output_path)


def model_fingerprint(urdf_path: Path, curobo_version: str) -> str:
    """Hash inputs that require refitting the collision spheres."""

    digest = hashlib.sha256()
    digest.update(urdf_path.read_bytes())
    digest.update(curobo_version.encode("utf-8"))
    digest.update(
        b"tool=tool0;gripper_open=0.0;collision_mesh=true;"
        b"clip_base_z=0.0;samples=200;seed=42"
    )
    return digest.hexdigest()


def build_robot_config(
    urdf_path: Path,
    asset_root: Path,
    config_path: Path,
) -> dict[str, Any]:
    """Fit collision spheres and save a six-DOF planning configuration."""

    np.random.seed(42)
    torch.manual_seed(42)
    builder = RobotBuilder(
        urdf_path=str(urdf_path),
        asset_path=str(asset_root),
        tool_frames=[TOOL_FRAME],
    )
    builder.fit_collision_spheres(
        sphere_density=1.0,
        use_collision_mesh=True,
        clip_links={"base_link_inertia": ("z", 0.0)},
    )
    builder.compute_collision_matrix(num_samples=200, seed=42)
    config = builder.build()
    config.lock_joints = {GRIPPER_JOINT_NAME: GRIPPER_OPEN_RAD}

    temporary_path = config_path.with_suffix(".yml.tmp")
    builder.save(config, str(temporary_path))
    temporary_path.replace(config_path)
    return {
        "collision_links": len(builder.collision_link_names),
        "collision_spheres": builder.num_spheres,
    }


def prepare_robot_model(
    repo: Path,
    cache_dir: Path,
    rebuild: bool,
) -> tuple[Path, dict[str, Any]]:
    """Create or reuse the cache-local cuRobo robot configuration."""

    cache_dir.mkdir(parents=True, exist_ok=True)
    asset_root = cache_dir / "assets"
    asset_root.mkdir(exist_ok=True)
    ensure_asset_link(asset_root, "ur_description", package_share("ur_description"))
    ensure_asset_link(
        asset_root,
        "robotiq_description",
        package_share("robotiq_description"),
    )

    urdf_path = cache_dir / "ur5_robotiq.urdf"
    config_path = cache_dir / "ur5_robotiq_curobo.yml"
    stamp_path = cache_dir / "model_stamp.json"
    generate_urdf(repo, urdf_path)

    curobo_version = getattr(curobo, "__version__", "unknown")
    fingerprint = model_fingerprint(urdf_path, curobo_version)
    old_stamp: dict[str, Any] = {}
    if stamp_path.exists():
        old_stamp = json.loads(stamp_path.read_text(encoding="utf-8"))

    cache_hit = (
        not rebuild
        and config_path.exists()
        and old_stamp.get("fingerprint") == fingerprint
    )
    build_metrics: dict[str, Any] = {}
    if not cache_hit:
        print("Building cuRobo collision model (first run takes about 30 seconds)...")
        build_metrics = build_robot_config(urdf_path, asset_root, config_path)
        stamp = {
            "fingerprint": fingerprint,
            "curobo_version": curobo_version,
            **build_metrics,
        }
        stamp_path.write_text(json.dumps(stamp, indent=2) + "\n", encoding="utf-8")

    return config_path, {"cache_hit": cache_hit, **build_metrics}


def load_initial_arm_positions(repo: Path) -> dict[str, float]:
    """Read the same UR5 start configuration used by the Jazzy FEM experiment."""

    path = repo / "config" / "jazzy_fem_initial_positions.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    missing = [name for name in ARM_JOINT_NAMES if name not in data]
    if missing:
        raise RuntimeError(f"Initial-position file is missing joints: {missing}")
    return {name: float(data[name]) for name in ARM_JOINT_NAMES}


def load_scene(
    repo: Path,
    profile_name: str,
) -> tuple[Scene, dict[str, Any], list[float]]:
    """Translate the existing obstacle profile to a cuRobo scene."""

    path = repo / "config" / "obstacles.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    profiles = data["scene_profiles"]
    if profile_name not in profiles:
        raise RuntimeError(
            f"Unknown scene profile {profile_name!r}; choices: {sorted(profiles)}"
        )
    profile = profiles[profile_name]
    raw_object_center = profile["object_center_m"]
    if any(value is None for value in raw_object_center):
        raise RuntimeError(
            f"Scene profile {profile_name!r} needs a measured object z position "
            "before it can be used by cuRobo."
        )
    object_center = [float(value) for value in raw_object_center]

    cuboid_data: dict[str, dict[str, list[float]]] = {
        "floor": {
            "dims": [3.0, 3.0, 0.02],
            "pose": [0.0, 0.0, -0.011, 1.0, 0.0, 0.0, 0.0],
        }
    }
    if profile["obstacles_enabled"]:
        obstacle = next(
            box for box in data["boxes"] if box["id"] == profile["box_id"]
        )
        cuboid_data[obstacle["id"]] = {
            "dims": [float(value) for value in obstacle["size_m"]],
            "pose": [
                *[float(value) for value in obstacle["center_m"]],
                1.0,
                0.0,
                0.0,
                0.0,
            ],
        }
    cuboids = [
        Cuboid(name=name, dims=value["dims"], pose=value["pose"])
        for name, value in cuboid_data.items()
    ]
    return Scene(cuboid=cuboids), {"cuboid": cuboid_data}, object_center


def quaternion_wxyz_from_rpy_degrees(
    roll_degrees: float,
    pitch_degrees: float,
    yaw_degrees: float,
) -> np.ndarray:
    """Return a wxyz quaternion using the same fixed-axis RPY convention as MoveIt."""

    roll = math.radians(roll_degrees) * 0.5
    pitch = math.radians(pitch_degrees) * 0.5
    yaw = math.radians(yaw_degrees) * 0.5
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=np.float64,
    )


def rotate_vector_by_quaternion(vector: np.ndarray, quaternion_wxyz: np.ndarray) -> np.ndarray:
    """Rotate a 3-vector by a normalized wxyz quaternion."""

    w = quaternion_wxyz[0]
    xyz = quaternion_wxyz[1:]
    return (
        2.0 * np.dot(xyz, vector) * xyz
        + (w * w - np.dot(xyz, xyz)) * vector
        + 2.0 * w * np.cross(xyz, vector)
    )


def pregrasp_target(object_center: list[float]) -> tuple[np.ndarray, np.ndarray]:
    """Reproduce the default MoveIt pre-grasp tool0 target."""

    orientation = quaternion_wxyz_from_rpy_degrees(180.0, 0.0, 90.0)
    local_tool_offset = np.asarray([0.0, 0.0, 0.109], dtype=np.float64)
    offset_world = rotate_vector_by_quaternion(local_tool_offset, orientation)
    position = np.asarray(object_center, dtype=np.float64) - offset_world
    position[2] += 0.150
    return position, orientation


def make_initial_joint_state(
    robot: Kinematics,
    initial_positions: dict[str, float],
) -> JointState:
    """Create a name-checked CUDA joint state for the six UR5 joints."""

    if tuple(robot.joint_names) != ARM_JOINT_NAMES:
        raise RuntimeError(
            "cuRobo active joints do not match the six UR5 arm joints: "
            f"{robot.joint_names}"
        )
    position = torch.tensor(
        [initial_positions[name] for name in robot.joint_names],
        device="cuda",
        dtype=torch.float32,
    )
    return JointState.from_position(position, joint_names=robot.joint_names)


def create_pick_planner(
    config_path: Path,
    scene_model: dict[str, Any],
) -> MotionPlanner:
    """Create and warm up the collision-aware CUDA motion planner."""

    planner_config = MotionPlannerCfg.create(
        robot=str(config_path),
        scene_model=scene_model,
        interpolation_dt=0.025,
    )
    planner = MotionPlanner(planner_config)
    print("Warming up cuRobo motion planner...")
    planner.warmup(enable_graph=True, num_warmup_iterations=3)
    return planner


def plan_cable_pick(
    planner: MotionPlanner,
    start_state: JointState,
    object_center: list[float],
):
    """Plan transit, vertical approach, and world-up lift for the cable tip."""

    pregrasp_position, orientation = pregrasp_target(object_center)
    grasp_position = pregrasp_position.copy()
    grasp_position[2] -= APPROACH_DISTANCE_M
    grasp_pose = Pose.from_list([*grasp_position.tolist(), *orientation.tolist()])
    goal = GoalToolPose.from_poses(
        {TOOL_FRAME: grasp_pose},
        ordered_tool_frames=[TOOL_FRAME],
        num_goalset=1,
    )
    started = time.monotonic()
    result = planner.plan_grasp(
        goal,
        start_state,
        grasp_approach_axis="z",
        grasp_approach_offset=-APPROACH_DISTANCE_M,
        grasp_approach_in_tool_frame=True,
        grasp_lift_axis="z",
        grasp_lift_offset=LIFT_DISTANCE_M,
        grasp_lift_in_tool_frame=False,
        plan_approach_to_grasp=True,
        plan_grasp_to_lift=True,
    )
    elapsed = time.monotonic() - started
    return result, elapsed


def effective_trajectory(trajectory: JointState, last_tstep: torch.Tensor) -> JointState:
    """Trim cuRobo's preallocated interpolation buffer to its valid samples."""

    end_index = int(last_tstep.reshape(-1)[0].item())
    return trim_joint_state_trajectory(trajectory, 0, end_index)


def pick_result_summary(result, elapsed: float) -> dict[str, Any]:
    """Return a compact, JSON-safe grasp planning summary."""

    phases = (
        ("approach", result.approach_interpolated_trajectory, result.approach_interpolated_last_tstep),
        ("grasp", result.grasp_interpolated_trajectory, result.grasp_interpolated_last_tstep),
        ("lift", result.lift_interpolated_trajectory, result.lift_interpolated_last_tstep),
    )
    summary: dict[str, Any] = {
        "success": bool(result.success.any().item()),
        "approach_success": bool(result.approach_success.any().item()),
        "grasp_success": bool(result.grasp_success.any().item()),
        "lift_success": bool(result.lift_success.any().item()),
        "status": result.status,
        "planning_elapsed_s": elapsed,
    }
    for name, trajectory, last_tstep in phases:
        summary[f"{name}_waypoints"] = (
            None
            if trajectory is None or last_tstep is None
            else int(effective_trajectory(trajectory, last_tstep).position.shape[-2])
        )
    return summary


def cable_curve(anchor: np.ndarray, endpoint: np.ndarray) -> np.ndarray:
    """Create a visual-only cable curve that stays near the floor before rising."""

    parameter = np.linspace(0.0, 1.0, 24, dtype=np.float64)
    points = (1.0 - parameter[:, None]) * anchor + parameter[:, None] * endpoint
    height = max(float(endpoint[2] - anchor[2]), 0.0)
    points[:, 2] = anchor[2] + height * parameter**4
    return points


def verify_cuda_and_source(repo: Path) -> dict[str, Any]:
    """Reject CPU fallback and accidental import of the old cuRobo checkout."""

    if not torch.cuda.is_available():
        raise RuntimeError(
            "CUDA is unavailable. This preflight intentionally refuses CPU fallback; "
            "run it through scripts/run_curobo_v2_viser.sh on the GPU host."
        )
    expected_root = (repo / "curobo").resolve()
    imported_path = Path(curobo.__file__).resolve()
    if expected_root not in imported_path.parents:
        raise RuntimeError(
            f"Imported cuRobo from {imported_path}, expected the V2 checkout under "
            f"{expected_root}."
        )
    return {
        "device": "cuda:0",
        "gpu": torch.cuda.get_device_name(0),
        "torch": torch.__version__,
        "torch_cuda": torch.version.cuda,
        "curobo": getattr(curobo, "__version__", "unknown"),
        "curobo_path": str(imported_path),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Offline cuRoboV2 + Viser preflight for the existing UR5 experiment."
    )
    parser.add_argument(
        "--cache-dir",
        type=Path,
        default=Path.home() / ".cache" / "ur5_moveit_demo" / "curobo_v2",
    )
    parser.add_argument("--rebuild-model", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--plan-pick",
        action="store_true",
        help="Plan the three-phase cable pick; combine with --headless for validation.",
    )
    parser.add_argument(
        "--auto-pick",
        action="store_true",
        help="Automatically plan and animate one visual cable pick after Viser starts.",
    )
    parser.add_argument("--show-spheres", action="store_true")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--animation-speed", type=float, default=1.0)
    parser.add_argument("--scene-profile", default="step_c_cable_wall")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo = repository_root()
    runtime = verify_cuda_and_source(repo)
    config_path, cache_status = prepare_robot_model(
        repo,
        args.cache_dir.expanduser().resolve(),
        args.rebuild_model,
    )

    robot = Kinematics(KinematicsCfg.from_robot_yaml_file(str(config_path)))
    initial_positions = load_initial_arm_positions(repo)
    joint_state = make_initial_joint_state(robot, initial_positions)
    kinematics_state = robot.compute_kinematics(joint_state)
    tool_pose = kinematics_state.tool_poses[TOOL_FRAME]
    scene, scene_model, object_center = load_scene(repo, args.scene_profile)
    target_position, target_orientation = pregrasp_target(object_center)

    summary = {
        **runtime,
        **cache_status,
        "robot_config": str(config_path),
        "dof": robot.get_dof(),
        "joint_names": robot.joint_names,
        "locked_joints": {GRIPPER_JOINT_NAME: GRIPPER_OPEN_RAD},
        "tool_frame": TOOL_FRAME,
        "initial_tool_position_m": tool_pose.position.detach().cpu().reshape(-1).tolist(),
        "scene_profile": args.scene_profile,
        "object_center_m": object_center,
        "pregrasp_tool_position_m": target_position.tolist(),
        "finite_state": bool(torch.isfinite(tool_pose.position).all().item()),
    }
    print("CUROBO_V2_UR5_PREFLIGHT " + json.dumps(summary))

    planner = None
    if args.plan_pick or not args.headless:
        planner = create_pick_planner(config_path, scene_model)

    if args.plan_pick:
        result, elapsed = plan_cable_pick(planner, joint_state.unsqueeze(0), object_center)
        plan_summary = pick_result_summary(result, elapsed)
        print("CUROBO_V2_UR5_PICK_PLAN " + json.dumps(plan_summary))
        if args.headless:
            return 0 if plan_summary["success"] else 2

    if args.headless:
        return 0

    if args.animation_speed <= 0.0:
        raise RuntimeError("--animation-speed must be greater than zero")

    visualizer = ViserVisualizer(
        content_path=ContentPath(robot_config_file=str(config_path)),
        connect_ip="0.0.0.0",
        connect_port=args.port,
        add_control_frames=True,
        visualize_robot_spheres=args.show_spheres,
        add_robot_to_scene=True,
    )
    visualizer.set_joint_state(joint_state)
    visualizer.add_scene(scene)
    cable_marker = visualizer._server.scene.add_icosphere(
        name="/targets/cable_pick_center",
        position=np.asarray(object_center),
        radius=0.009,
        color=(255, 170, 0),
    )
    cable_endpoint = np.asarray(object_center, dtype=np.float64)
    cable_anchor = cable_endpoint + np.asarray([-CABLE_LENGTH_M, 0.0, 0.0])
    cable_handle = visualizer._server.scene.add_spline_catmull_rom(
        name="/cable/visual_only",
        points=cable_curve(cable_anchor, cable_endpoint),
        line_width=5.0,
        color=(255, 130, 20),
    )
    target_handle = visualizer._control_frames[TOOL_FRAME]
    target_handle.position = target_position
    target_handle.wxyz = target_orientation

    status_handle = visualizer._server.gui.add_markdown(
        "**Ready.** Click **Plan + pick cable**. The cable attachment is visual only."
    )
    pick_button = visualizer._server.gui.add_button("Plan + pick cable", color="green")
    reset_button = visualizer._server.gui.add_button("Reset", color="red")
    state_lock = threading.Lock()
    busy = False

    def arm_only_state(state: JointState) -> JointState:
        """Drop cuRobo's locked gripper joint from an interpolated state."""

        return state.reorder(list(ARM_JOINT_NAMES))

    def set_visual_state(arm_state: JointState, gripper_position: float) -> None:
        active_state = arm_only_state(arm_state)
        arm_values = active_state.position.detach().cpu().reshape(
            -1, len(ARM_JOINT_NAMES)
        )[0]
        value_by_name = dict(zip(ARM_JOINT_NAMES, arm_values.tolist()))
        values = [
            gripper_position
            if name == GRIPPER_JOINT_NAME
            else value_by_name[name]
            for name in visualizer.joint_names
        ]
        visualizer._viser_urdf.update_cfg(np.asarray(values, dtype=np.float64))
        if args.show_spheres:
            visualizer.update_robot_spheres(active_state)

    def update_attached_cable(arm_state: JointState) -> None:
        state = planner.kinematics.compute_kinematics(arm_only_state(arm_state))
        tool_position = (
            state.tool_poses[TOOL_FRAME].position.detach().cpu().reshape(-1, 3)[0].numpy()
        )
        endpoint = tool_position + np.asarray([0.0, 0.0, -GRASP_CENTER_BELOW_TOOL_M])
        cable_marker.position = endpoint
        cable_handle.positions = cable_curve(cable_anchor, endpoint)

    def animate_phase(
        trajectory: JointState,
        last_tstep: torch.Tensor,
        gripper_position: float,
        cable_attached: bool,
    ) -> JointState:
        active = effective_trajectory(trajectory, last_tstep)
        positions = active.position.reshape(-1, active.position.shape[-1])
        delay = 0.025 / args.animation_speed
        final_state = None
        for position in positions:
            final_state = JointState.from_position(
                position.unsqueeze(0),
                joint_names=active.joint_names,
            )
            set_visual_state(final_state, gripper_position)
            if cable_attached:
                update_attached_cable(final_state)
            time.sleep(delay)
        if final_state is None:
            raise RuntimeError("cuRobo returned an empty trajectory")
        return final_state

    def reset_visuals() -> None:
        set_visual_state(joint_state, GRIPPER_OPEN_RAD)
        cable_marker.position = cable_endpoint
        cable_handle.positions = cable_curve(cable_anchor, cable_endpoint)
        target_handle.position = target_position
        target_handle.wxyz = target_orientation

    def run_visual_pick() -> None:
        nonlocal busy
        with state_lock:
            if busy:
                return
            busy = True
        try:
            reset_visuals()
            status_handle.content = "**Planning on CUDA...** No ROS command is being sent."
            result, elapsed = plan_cable_pick(
                planner,
                joint_state.unsqueeze(0),
                object_center,
            )
            plan_summary = pick_result_summary(result, elapsed)
            print("CUROBO_V2_UR5_PICK_PLAN " + json.dumps(plan_summary))
            if not plan_summary["success"]:
                status_handle.content = f"**Planning failed:** {result.status}"
                return

            status_handle.content = "**1/3 Approach:** collision-aware transit to pre-grasp."
            animate_phase(
                result.approach_interpolated_trajectory,
                result.approach_interpolated_last_tstep,
                GRIPPER_OPEN_RAD,
                False,
            )
            status_handle.content = "**2/3 Grasp:** vertical approach to the cable."
            grasp_state = animate_phase(
                result.grasp_interpolated_trajectory,
                result.grasp_interpolated_last_tstep,
                GRIPPER_OPEN_RAD,
                False,
            )
            for gripper_position in np.linspace(
                GRIPPER_OPEN_RAD,
                GRIPPER_CLOSED_RAD,
                20,
            ):
                set_visual_state(grasp_state, float(gripper_position))
                time.sleep(0.025 / args.animation_speed)
            update_attached_cable(grasp_state)

            status_handle.content = "**3/3 Lift:** cable visually attached; lifting world +Z."
            animate_phase(
                result.lift_interpolated_trajectory,
                result.lift_interpolated_last_tstep,
                GRIPPER_CLOSED_RAD,
                True,
            )
            status_handle.content = (
                "**Visual pick complete.** Robot path used cuRobo collision checking; "
                "cable deformation is not physics yet."
            )
            print("CUROBO_V2_UR5_PICK_ANIMATION_COMPLETE")
        except Exception as error:
            status_handle.content = f"**Pick error:** `{error}`"
            print(f"CUROBO_V2_UR5_PICK_ERROR {error}", file=sys.stderr)
        finally:
            with state_lock:
                busy = False

    def on_pick(_event) -> None:
        threading.Thread(target=run_visual_pick, daemon=True).start()

    def on_reset(_event) -> None:
        nonlocal busy
        with state_lock:
            if busy:
                return
        reset_visuals()
        status_handle.content = "**Reset.** Ready to plan another visual pick."

    pick_button.on_click(on_pick)
    reset_button.on_click(on_reset)

    print(f"Viser is running at http://localhost:{args.port}")
    print("Orange cable = visual-only cable; tool0 gizmo = MoveIt-equivalent pre-grasp target.")
    print("Click 'Plan + pick cable' to plan and animate on CUDA.")
    print("No trajectory is published to ROS or hardware. Press Ctrl+C to stop.")
    if args.auto_pick:
        threading.Thread(target=run_visual_pick, daemon=True).start()
    started = time.monotonic()
    try:
        while args.duration <= 0.0 or time.monotonic() - started < args.duration:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping Viser.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(f"CUROBO_V2_UR5_ERROR {error}", file=sys.stderr)
        raise SystemExit(1)
