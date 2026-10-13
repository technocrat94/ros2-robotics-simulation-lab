#!/usr/bin/env python3
"""Combine Step C MoveIt route evidence with Newton grasp measurements."""

import argparse
import json
import re
from pathlib import Path


ROUTE_PATTERN = re.compile(
    r'PLANNED_ROUTE_METRICS stage="([^"]+)" samples=(\d+) '
    r'max_end_effector_height_m=([-0-9.]+) '
    r'min_robot_box_distance_m=([-0-9.]+) obstacle_id=([^\s]+)'
)


def route_metrics(text):
    matches = ROUTE_PATTERN.findall(text)
    if not matches:
        return None
    stage, samples, height, distance, obstacle_id = matches[-1]
    return {
        "stage": stage,
        "samples": int(samples),
        "max_end_effector_height_m": float(height),
        "min_robot_wall_distance_m": float(distance),
        "obstacle_id": obstacle_id,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=("c0", "c1", "c2"), required=True)
    parser.add_argument("--pick-log", type=Path, required=True)
    parser.add_argument("--endpoint-json", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tested-commit", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--padding", type=float, required=True)
    args = parser.parse_args()

    text = args.pick_log.read_text(encoding="utf-8")
    expected_scene = (
        f"SCENE_CONFIG profile={args.profile} "
        f"obstacles_enabled={'true' if args.case != 'c0' else 'false'} "
        f"robot_padding_m={args.padding:.6f}"
    )
    common_checks = {
        "scene_profile_and_padding_logged": expected_scene in text,
    }

    if args.case == "c2":
        planning_failed = bool(re.search(
            r"(Absolute pre-grasp planning failed|No motion plan found|PLANNING_FAILED)",
            text,
        ))
        no_robot_command = not any(marker in text for marker in (
            "Execute request accepted",
            "Sending gripper command",
            "ABSOLUTE POSITION PICK SUCCEEDED",
        ))
        checks = {
            **common_checks,
            "planning_failed": planning_failed,
            "plan_only_did_not_succeed": "PLAN_ONLY_SUCCEEDED" not in text,
            "no_robot_command_sent": no_robot_command,
        }
        result = {
            "step": "C2_tall_wall_negative_control",
            "tested_commit": args.tested_commit,
            "scene_profile": args.profile,
            "moveit_robot_padding_m": args.padding,
            "planning_failed": planning_failed,
            "robot_command_sent": not no_robot_command,
            "route": route_metrics(text),
            "checks": checks,
            "pass": all(checks.values()),
        }
    else:
        if args.endpoint_json is None:
            parser.error("--endpoint-json is required for C0 and C1")
        endpoint = json.loads(args.endpoint_json.read_text(encoding="utf-8"))
        route = route_metrics(text)
        checks = {
            **common_checks,
            "moveit_pick_succeeded": "ABSOLUTE POSITION PICK SUCCEEDED" in text,
            "route_metrics_present": route is not None,
            "candidate_contact_grasp_pass": (
                endpoint.get("candidate_contact_grasp_pass") is True
            ),
            "tested_commit_matches": (
                endpoint.get("tested_commit") == args.tested_commit
            ),
            "endpoint_profile_matches": (
                endpoint.get("scene_profile") == args.profile
            ),
            "endpoint_padding_matches": abs(
                float(endpoint.get("moveit_robot_padding_m", -1.0)) - args.padding
            ) <= 1.0e-12,
        }
        if args.case == "c1":
            checks["zero_robot_wall_contacts"] = (
                endpoint.get("maximum_robot_obstacle_contacts") == 0
            )
        result = {
            "step": (
                "C0_moved_cable_without_wall"
                if args.case == "c0"
                else "C1_moved_cable_with_wall"
            ),
            "tested_commit": args.tested_commit,
            "scene_profile": args.profile,
            "moveit_robot_padding_m": args.padding,
            "candidate_contact_grasp_pass": endpoint.get(
                "candidate_contact_grasp_pass"
            ),
            "grasp_region_lift_m": endpoint.get("grasp_region_lift_m"),
            "release_drop_m": endpoint.get("release_drop_m"),
            "maximum_robot_obstacle_contacts": endpoint.get(
                "maximum_robot_obstacle_contacts"
            ),
            "object_obstacle_max_penetration_m": endpoint.get(
                "object_obstacle_max_penetration_m"
            ),
            "route": route,
            "endpoint_result": args.endpoint_json.name,
            "checks": checks,
            "pass": all(checks.values()),
        }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print("STEP_C_RESULT", json.dumps(result, sort_keys=True))
    print(f"STEP_C_RESULT_JSON {args.output}")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
