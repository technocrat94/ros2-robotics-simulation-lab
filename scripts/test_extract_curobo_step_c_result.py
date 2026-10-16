#!/usr/bin/env python3
"""Dependency-free checks for the cuRobo/Newton C1 result extractor."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile


REPO = Path(__file__).resolve().parents[1]
EXTRACTOR = REPO / "scripts" / "extract_curobo_step_c_result.py"
COMMIT = "0123456789abcdef"


PICK_LOG = """\
SCENE_CONFIG profile=step_c_cable_wall obstacles_enabled=true transit_padding_m=0.020000 contact_padding_m=0.000000 planning_time_s=10.000
USE_SAFE_PREGRASP_PATH true
CUROBO_RUNTIME {"device": "cuda:0"}
CUROBO_PLANNING_TIME segment=approach seconds=0.100000
CUROBO_PLANNING_TIME segment=grasp seconds=0.020000
CUROBO_PLANNING_TIME segment=lift seconds=0.030000
PLANNED_ROUTE_METRICS stage="curobo_pregrasp" samples=41 max_end_effector_height_m=0.394224 min_robot_box_distance_m=0.036070 obstacle_id=step_c_cable_wall
ACTIVE_PADDING stage=transit padding_m=0.020000
ACTIVE_PADDING stage=contact padding_m=0.000000
========== CUROBO FEM CABLE PICK SUCCEEDED ==========
"""


def run_case(directory: Path, endpoint: dict, expected_status: int) -> dict:
    pick_path = directory / "pick.log"
    endpoint_path = directory / "endpoint.json"
    output_path = directory / "combined.json"
    pick_path.write_text(PICK_LOG, encoding="utf-8")
    endpoint_path.write_text(json.dumps(endpoint), encoding="utf-8")
    result = subprocess.run(
        [
            sys.executable,
            str(EXTRACTOR),
            "--pick-log",
            str(pick_path),
            "--endpoint-json",
            str(endpoint_path),
            "--output",
            str(output_path),
            "--tested-commit",
            COMMIT,
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == expected_status, result.stdout + result.stderr
    return json.loads(output_path.read_text(encoding="utf-8"))


def main() -> None:
    endpoint = {
        "tested_commit": COMMIT,
        "device": "cuda:0",
        "scene_profile": "step_c_cable_wall",
        "moveit_transit_padding_m": 0.02,
        "moveit_contact_padding_m": 0.0,
        "candidate_contact_grasp_pass": True,
        "grasp_region_lift_m": 0.12,
        "release_drop_m": 0.12,
        "maximum_robot_obstacle_contacts": 0,
        "maximum_robot_obstacle_candidate_contacts": 82,
        "maximum_robot_obstacle_loaded_contacts": 0,
        "maximum_robot_obstacle_penetrating_contacts": 0,
        "object_obstacle_max_penetration_m": 0.0,
    }
    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        passed = run_case(directory, endpoint, 0)
        assert passed["pass"] is True
        assert passed["route"]["min_robot_wall_distance_m"] == 0.03607

        endpoint["maximum_robot_obstacle_loaded_contacts"] = 1
        failed = run_case(directory, endpoint, 1)
        assert failed["pass"] is False
        assert failed["checks"]["zero_loaded_robot_wall_contacts"] is False
    print("cuRobo Step C result extractor: PASS")


if __name__ == "__main__":
    main()
