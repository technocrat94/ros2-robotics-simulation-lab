#!/usr/bin/env python3
"""Extract Step B enabled/disabled/tall-obstacle planning evidence."""

import argparse
import json
import re
from pathlib import Path


def section(text, begin, end):
    return text.split(begin, 1)[1].split(end, 1)[0]


def route_metrics(text):
    match = re.search(
        r'PLANNED_ROUTE_METRICS stage="[^"]+" samples=(\d+) '
        r'max_end_effector_height_m=([-0-9.]+) '
        r'min_robot_box_distance_m=([-0-9.]+)', text)
    if not match:
        return None
    return {
        "samples": int(match.group(1)),
        "max_end_effector_height_m": float(match.group(2)),
        "min_robot_box_distance_m": float(match.group(3)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tested-commit", required=True)
    args = parser.parse_args()
    text = args.log.read_text(encoding="utf-8")
    disabled = section(text, "STEP_B_NEGATIVE_CONTROL_DISABLED", "STEP_B_ENABLED_SAME_TARGET")
    enabled = section(text, "STEP_B_ENABLED_SAME_TARGET", "STEP_B_THREE_SPATIAL_STAGE_PREVIEW")
    staged = section(text, "STEP_B_THREE_SPATIAL_STAGE_PREVIEW", "STEP_B_TALL_OBSTACLE_NEGATIVE_CONTROL")
    tall = section(text, "STEP_B_TALL_OBSTACLE_NEGATIVE_CONTROL", "STEP_B_DISABLED_CLEANUP_DEFAULT_PICK")
    cleanup = text.split("STEP_B_DISABLED_CLEANUP_DEFAULT_PICK", 1)[1]
    disabled_metrics = route_metrics(disabled)
    enabled_ompl_metrics = route_metrics(enabled)
    staged_metrics = route_metrics(staged)
    enabled_metrics = staged_metrics
    status_match = re.search(r"TALL_OBSTACLE_LAUNCH_EXIT_STATUS (\d+)", tall)
    fractions = {stage: float(value) for stage, value in re.findall(
        r"SAFE_PREGRASP_STAGE_FRACTION stage=(raise|orient|translate|descend) prefix=([0-9.]+)%", staged)}
    route_changed = bool(disabled_metrics and enabled_metrics and (
        abs(disabled_metrics["max_end_effector_height_m"] - enabled_metrics["max_end_effector_height_m"]) > 1e-4
        or abs(disabled_metrics["min_robot_box_distance_m"] - enabled_metrics["min_robot_box_distance_m"]) > 1e-4))
    checks = {
        "disabled_control_planned": "PLAN_ONLY_SUCCEEDED" in disabled,
        "enabled_safe_route_planned": "SAFE_PREGRASP_PLAN_ONLY_SUCCEEDED" in staged,
        "route_metrics_present": disabled_metrics is not None and enabled_metrics is not None,
        "route_changed_because_of_box": route_changed,
        "enabled_route_is_collision_free": bool(enabled_metrics and enabled_metrics["min_robot_box_distance_m"] >= -1e-6),
        "staged_route_accepted": "SAFE_PREGRASP_PLAN_ONLY_SUCCEEDED" in staged,
        "three_spatial_stages_100_percent": all(fractions.get(name) == 100.0 for name in ("raise", "translate", "descend")),
        "tall_obstacle_planning_failed": "TALL_OBSTACLE_PLANNING_FAILED true" in tall,
        "persistent_obstacle_removed": bool(re.search(
            r"PLANNING_SCENE_OBSTACLE_REMOVED id=grasp_route_box .*confirmed_absent=true", cleanup)),
        "disabled_scene_clean": "PLANNING_SCENE_OBSTACLES_DISABLED scene_clean=true" in cleanup,
    }
    result = {
        "step": "B_pure_moveit_planning_controls", "tested_commit": args.tested_commit,
        "same_target_x_m": 0.20, "disabled_route": disabled_metrics,
        "enabled_route": enabled_metrics,
        "enabled_ompl_route": enabled_ompl_metrics,
        "enabled_ompl_plan_succeeded": "PLAN_ONLY_SUCCEEDED" in enabled,
        "safe_staged_route": staged_metrics,
        "staged_fractions_percent": fractions,
        "tall_obstacle_plan_exit_status": int(status_match.group(1)) if status_match else None,
        "checks": checks, "pass": all(checks.values()),
    }
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"RIGID_OBSTACLE_PLAN_RESULT_JSON {args.output}")
    print("RIGID_OBSTACLE_PLAN_RESULT", json.dumps(result, sort_keys=True))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
