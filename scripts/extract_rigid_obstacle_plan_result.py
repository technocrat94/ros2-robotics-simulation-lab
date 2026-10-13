#!/usr/bin/env python3
"""Extract Step B obstacle-planning evidence from its ROS launch log."""

import argparse
import json
import re
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tested-commit", required=True)
    args = parser.parse_args()

    text = args.log.read_text(encoding="utf-8")
    fractions = {
        stage: float(value)
        for stage, value in re.findall(
            r"SAFE_PREGRASP_STAGE_FRACTION stage=(raise|orient|translate|descend) "
            r"prefix=([0-9.]+)%",
            text,
        )
    }
    spatial = {stage: fractions.get(stage) for stage in ("raise", "translate", "descend")}
    checks = {
        "obstacle_added": "PLANNING_SCENE_OBSTACLE_ADDED" in text,
        "ompl_plan_succeeded": "Absolute pre-grasp PLAN_ONLY_SUCCEEDED" in text,
        "three_spatial_stages_100_percent": all(
            value == 100.0 for value in spatial.values()
        ),
        "orientation_stage_100_percent": fractions.get("orient") == 100.0,
        "complete_safe_route_100_percent":
            "Safe pre-grasp Cartesian path completed: 100.0%" in text,
        "safe_route_accepted": "SAFE_PREGRASP_PLAN_ONLY_SUCCEEDED" in text,
        "no_robot_command_sent": text.count("no robot command was sent") >= 2,
    }
    result = {
        "step": "B_pure_moveit_planning",
        "tested_commit": args.tested_commit,
        "obstacles_enabled_for_test": True,
        "object_target_x_m": 0.20,
        "safe_transit_margin_m": 0.15,
        "spatial_stage_fractions_percent": spatial,
        "orientation_stage_fraction_percent": fractions.get("orient"),
        "checks": checks,
        "pass": all(checks.values()),
    }
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"RIGID_OBSTACLE_PLAN_RESULT_JSON {args.output}")
    print("RIGID_OBSTACLE_PLAN_RESULT", json.dumps(result, sort_keys=True))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
