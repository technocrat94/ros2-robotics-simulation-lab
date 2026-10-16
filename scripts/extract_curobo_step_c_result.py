#!/usr/bin/env python3
"""Combine cuRobo planning evidence with the unchanged Newton C1 result."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path


ROUTE_PATTERN = re.compile(
    r'PLANNED_ROUTE_METRICS stage="([^"]+)" samples=(\d+) '
    r"max_end_effector_height_m=([-0-9.]+) "
    r"min_robot_box_distance_m=([-0-9.]+) obstacle_id=([^\s]+)"
)
PLANNING_TIME_PATTERN = re.compile(
    r"CUROBO_PLANNING_TIME segment=(approach|grasp|lift) "
    r"seconds=([-+0-9.eE]+)"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pick-log", type=Path, required=True)
    parser.add_argument("--endpoint-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tested-commit", required=True)
    args = parser.parse_args()

    text = args.pick_log.read_text(encoding="utf-8")
    endpoint = json.loads(args.endpoint_json.read_text(encoding="utf-8"))
    route_matches = ROUTE_PATTERN.findall(text)
    route = None
    if route_matches:
        stage, samples, height, distance, obstacle_id = route_matches[-1]
        route = {
            "stage": stage,
            "samples": int(samples),
            "max_end_effector_height_m": float(height),
            "min_robot_wall_distance_m": float(distance),
            "obstacle_id": obstacle_id,
        }
    planning_times = {
        segment: float(seconds)
        for segment, seconds in PLANNING_TIME_PATTERN.findall(text)
    }
    expected_scene = (
        "SCENE_CONFIG profile=step_c_cable_wall obstacles_enabled=true "
        "transit_padding_m=0.020000 contact_padding_m=0.000000 "
        "planning_time_s=10.000"
    )
    checks = {
        "curobo_pick_succeeded": (
            "CUROBO FEM CABLE PICK SUCCEEDED" in text
        ),
        "cuda_0_logged": '"device": "cuda:0"' in text,
        "scene_profile_and_padding_logged": expected_scene in text,
        "safe_pregrasp_path_logged": "USE_SAFE_PREGRASP_PATH true" in text,
        "transit_padding_stage_logged": (
            "ACTIVE_PADDING stage=transit padding_m=0.020000" in text
        ),
        "contact_padding_stage_logged": (
            "ACTIVE_PADDING stage=contact padding_m=0.000000" in text
        ),
        "all_segment_planning_times_logged": (
            set(planning_times) == {"approach", "grasp", "lift"}
            and all(
                math.isfinite(value) and value >= 0.0
                for value in planning_times.values()
            )
        ),
        "route_metrics_present": route is not None,
        "candidate_contact_grasp_pass": (
            endpoint.get("candidate_contact_grasp_pass") is True
        ),
        "tested_commit_matches": endpoint.get("tested_commit") == args.tested_commit,
        "endpoint_cuda_0": endpoint.get("device") == "cuda:0",
        "endpoint_profile_matches": (
            endpoint.get("scene_profile") == "step_c_cable_wall"
        ),
        "endpoint_transit_padding_matches": (
            abs(float(endpoint.get("moveit_transit_padding_m", -1.0)) - 0.02)
            <= 1.0e-12
        ),
        "endpoint_contact_padding_matches": (
            abs(float(endpoint.get("moveit_contact_padding_m", -1.0)))
            <= 1.0e-12
        ),
        "zero_robot_wall_contacts": (
            endpoint.get("maximum_robot_obstacle_contacts") == 0
        ),
        "zero_loaded_robot_wall_contacts": (
            endpoint.get("maximum_robot_obstacle_loaded_contacts") == 0
        ),
        "zero_penetrating_robot_wall_contacts": (
            endpoint.get("maximum_robot_obstacle_penetrating_contacts") == 0
        ),
    }
    result = {
        "step": "C1_curobo_planner_newton_fem_cable_wall",
        "planner": "curobo_v2",
        "tested_commit": args.tested_commit,
        "device": endpoint.get("device"),
        "scene_profile": endpoint.get("scene_profile"),
        "planning_time_by_segment_s": planning_times,
        "candidate_contact_grasp_pass": endpoint.get(
            "candidate_contact_grasp_pass"
        ),
        "grasp_region_lift_m": endpoint.get("grasp_region_lift_m"),
        "release_drop_m": endpoint.get("release_drop_m"),
        "maximum_robot_obstacle_contacts": endpoint.get(
            "maximum_robot_obstacle_contacts"
        ),
        "maximum_robot_obstacle_candidate_contacts": endpoint.get(
            "maximum_robot_obstacle_candidate_contacts"
        ),
        "maximum_robot_obstacle_loaded_contacts": endpoint.get(
            "maximum_robot_obstacle_loaded_contacts"
        ),
        "maximum_robot_obstacle_penetrating_contacts": endpoint.get(
            "maximum_robot_obstacle_penetrating_contacts"
        ),
        "object_obstacle_max_penetration_m": endpoint.get(
            "object_obstacle_max_penetration_m"
        ),
        "route": route,
        "pick_log": args.pick_log.name,
        "endpoint_result": args.endpoint_json.name,
        "checks": checks,
        "pass": all(checks.values()),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print("CUROBO_STEP_C_RESULT " + json.dumps(result, sort_keys=True))
    print(f"CUROBO_STEP_C_RESULT_JSON {args.output}")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
