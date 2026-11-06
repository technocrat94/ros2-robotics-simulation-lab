#!/usr/bin/env python3
"""Persistent Newton camera worker using JSON lines and small NPZ replies."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

import newton

import newton_sim_depth_cloud_viser as camera_common
from newton_camera_static_capture import StaticObstacleCamera


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path.home() / "newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf",
    )
    parser.add_argument(
        "--mount-config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config/sim_camera_mount.yaml",
    )
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    mount, _ = camera_common.read_mount_config(args.mount_config)
    scenes: dict[tuple[float, ...], StaticObstacleCamera] = {}
    print(
        "NEWTON_CAMERA_WORKER_READY "
        + json.dumps({"newton": newton.__version__, "mount": mount.__dict__}),
        flush=True,
    )
    for line in sys.stdin:
        request = json.loads(line)
        if request.get("command") == "shutdown":
            print("NEWTON_CAMERA_WORKER_STOPPED", flush=True)
            return
        center = np.asarray(request["center_m"], dtype=np.float64)
        dims = np.asarray(request["dims_m"], dtype=np.float64)
        arm = np.asarray(request["arm_q"], dtype=np.float32)
        key = tuple(np.round(np.concatenate((center, dims)), 9))
        if key not in scenes:
            scenes[key] = StaticObstacleCamera(
                args.urdf, mount, center, dims, args.device
            )
        started = time.perf_counter()
        capture = scenes[key].capture(arm)
        capture_ms = (time.perf_counter() - started) * 1000.0
        output = Path(request["output"])
        np.savez_compressed(
            output,
            points=capture["points"],
            camera_transform=capture["camera_transform"],
        )
        points = capture["points"]
        response = {
            "id": request["id"],
            "output": str(output),
            "points": int(len(points)),
            "highest_seen_z_m": float(np.max(points[:, 2])) if len(points) else None,
            "capture_ms": capture_ms,
            "conversion_ms": capture["conversion_ms"],
            "obstacle_pixels": capture["obstacle_pixels"],
            "border_touches": capture["border_touches"],
            "border_pixel_counts": capture["border_pixel_counts"],
            "robot_percent": capture["robot_percent"],
            "flange_m": capture["flange_m"],
            "wrist_1_center_m": capture["wrist_1_center_m"],
        }
        print("NEWTON_CAMERA_WORKER_RESPONSE " + json.dumps(response), flush=True)


if __name__ == "__main__":
    main()
