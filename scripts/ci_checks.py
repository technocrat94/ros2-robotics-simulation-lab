#!/usr/bin/env python3
"""GPU-free repository checks run by GitHub Actions.

These checks do not start ROS, MoveIt, or Newton. They catch broken syntax,
malformed configuration, license drift, and regressions in the gripper-width
calibration that the verified FEM ground pick depends on.
"""

import csv
import json
import py_compile
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
EXPECTED_LICENSE = "Apache-2.0"

# Verified FEM ground pick (Humble 2026-10-01, Jazzy 2026-10-02):
# 50 mm strip, 2.0 mm total compression.
EXPECTED_GRIPPER_RAD = 0.375145
EXPECTED_CLOSURE_DROP_MM = 10.578

failures = []


def tracked(*patterns):
    output = subprocess.run(
        ["git", "ls-files", *patterns],
        cwd=REPO, check=True, capture_output=True, text=True,
    ).stdout
    return [REPO / line for line in output.splitlines() if line]


def check(name, function, paths):
    count = 0
    for path in paths:
        try:
            function(path)
            count += 1
        except Exception as error:  # noqa: BLE001 - report every failure
            failures.append(f"{name}: {path.relative_to(REPO)}: {error}")
    print(f"[{name}] checked {count}/{len(paths)} files")


def compile_python(path):
    py_compile.compile(str(path), doraise=True)


def check_shell(path):
    subprocess.run(["bash", "-n", str(path)], check=True, capture_output=True)


class RosYamlLoader(yaml.SafeLoader):
    """SafeLoader that tolerates ROS tags such as ``!degrees`` in UR configs."""


RosYamlLoader.add_multi_constructor(
    "!", lambda loader, suffix, node: loader.construct_scalar(node)
    if isinstance(node, yaml.ScalarNode) else None
)


def load_yaml(path):
    with path.open(encoding="utf-8") as stream:
        list(yaml.load_all(stream, Loader=RosYamlLoader))


def load_json(path):
    with path.open(encoding="utf-8") as stream:
        json.load(stream)


def parse_xml(path):
    ET.parse(path)


def check_licenses():
    license_text = (REPO / "LICENSE").read_text(encoding="utf-8")
    if "Apache License" not in license_text or "Version 2.0" not in license_text:
        failures.append("license: LICENSE is not the Apache License 2.0 text")
    for package_xml in tracked("package.xml", "**/package.xml"):
        declared = ET.parse(package_xml).getroot().findtext("license")
        if declared != EXPECTED_LICENSE:
            failures.append(
                f"license: {package_xml.relative_to(REPO)} declares {declared!r}"
            )
    print("[license] LICENSE and package.xml files checked")


def check_gripper_solver():
    result = subprocess.run(
        [
            sys.executable, str(REPO / "scripts/gripper_width_solver.py"),
            "--object-width-mm", "50",
            "--total-compression-mm", "2.0",
            "--table", str(REPO / "config/gripper_kinematics_calibration.csv"),
        ],
        check=True, capture_output=True, text=True,
    ).stdout
    line = next(
        (row for row in result.splitlines() if row.startswith("GRIPPER_SOLUTION ")),
        None,
    )
    if line is None:
        failures.append("gripper: solver printed no GRIPPER_SOLUTION line")
        return
    solution = json.loads(line.split(" ", 1)[1])
    if abs(solution["gripper_command_rad"] - EXPECTED_GRIPPER_RAD) > 1.0e-5:
        failures.append(
            f"gripper: command {solution['gripper_command_rad']} rad, "
            f"expected {EXPECTED_GRIPPER_RAD}"
        )
    if abs(solution["closure_drop_mm"] - EXPECTED_CLOSURE_DROP_MM) > 1.0e-2:
        failures.append(
            f"gripper: closure drop {solution['closure_drop_mm']} mm, "
            f"expected {EXPECTED_CLOSURE_DROP_MM}"
        )
    print(
        f"[gripper] 50 mm / 2.0 mm -> {solution['gripper_command_rad']:.6f} rad, "
        f"drop {solution['closure_drop_mm']:.3f} mm"
    )


def check_cpp_calibration_table():
    cpp_path = REPO / "src/pick_at_position.cpp"
    csv_path = REPO / "config/gripper_kinematics_calibration.csv"
    cpp_text = cpp_path.read_text(encoding="utf-8")
    table_match = re.search(
        r"static constexpr std::array<Sample,\s*\d+> samples = \{\{(.*?)\}\};",
        cpp_text,
        re.DOTALL,
    )
    if table_match is None:
        failures.append("calibration-table: C++ sample table was not found")
        return
    cpp_rows = [
        tuple(float(value) for value in match)
        for match in re.findall(
            r"\{\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\}",
            table_match.group(1),
        )
    ]
    with csv_path.open(encoding="utf-8", newline="") as stream:
        csv_rows = list(csv.DictReader(stream))
    if len(cpp_rows) != len(csv_rows):
        failures.append(
            "calibration-table: row count differs: "
            f"C++={len(cpp_rows)} CSV={len(csv_rows)}"
        )
        return
    fields = ("command_rad", "estimated_pad_gap_m", "closure_drop_m")
    tolerance = 1.0e-9
    for index, (cpp_row, csv_row) in enumerate(zip(cpp_rows, csv_rows), start=1):
        for field, cpp_value in zip(fields, cpp_row):
            csv_value = float(csv_row[field])
            if abs(cpp_value - csv_value) > tolerance:
                failures.append(
                    f"calibration-table: row {index} {field} differs: "
                    f"C++={cpp_value:.10g} CSV={csv_value:.10g}"
                )
    print(
        f"[calibration-table] matched {len(cpp_rows)} C++/CSV rows "
        f"within {tolerance:.0e}"
    )


def check_obstacle_defaults():
    path = REPO / "config/obstacles.yaml"
    config = json.loads(path.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1:
        failures.append("obstacles: schema_version must be 1")
    if config.get("enabled") is not False:
        failures.append("obstacles: enabled must default to false")
    boxes = config.get("boxes", [])
    boxes_by_id = {box.get("id"): box for box in boxes}
    expected_grasp_route_box = {
        "id": "grasp_route_box",
        "frame_id": "world",
        "center_m": [0.30, 0.10915, 0.125],
        "size_m": [0.10, 0.24, 0.25],
    }
    box = boxes_by_id.get("grasp_route_box")
    if box is None:
        failures.append("obstacles: Steps A/B require grasp_route_box")
    else:
        for key, expected in expected_grasp_route_box.items():
            if box.get(key) != expected:
                failures.append(
                    f"obstacles: grasp_route_box {key} changed from {expected!r}"
                )
    for box in boxes:
        if len(box.get("center_m", [])) != 3:
            failures.append("obstacles: center_m must have three values")
        if len(box.get("size_m", [])) != 3 or any(
            not isinstance(value, (int, float)) or value < 0.01
            for value in box.get("size_m", [])
        ):
            failures.append(
                "obstacles: every box dimension must be at least 0.01 m"
            )
    profiles = config.get("scene_profiles", {})
    if config.get("default_scene_profile") != "default":
        failures.append("obstacles: default_scene_profile must be default")
    expected_profiles = {
        "default", "step_c_cable_no_wall", "step_c_cable_wall",
        "step_c_cable_tall_wall", "moving_wall",
    }
    if set(profiles) != expected_profiles:
        failures.append(
            f"obstacles: scene profiles differ from {sorted(expected_profiles)}"
        )
    for name, profile in profiles.items():
        center = profile.get("object_center_m", [])
        if len(center) != 3:
            failures.append(f"obstacles: profile {name} needs a 3D object center")
        if profile.get("box_id") not in boxes_by_id:
            failures.append(f"obstacles: profile {name} references an unknown box")
    expected_step_c = {
        "step_c_cable_no_wall": {
            "object_center_m": [0.487, 0.410, 0.004],
            "obstacles_enabled": False,
            "box_id": "step_c_cable_wall",
        },
        "step_c_cable_wall": {
            "object_center_m": [0.487, 0.410, 0.004],
            "obstacles_enabled": True,
            "box_id": "step_c_cable_wall",
        },
        "step_c_cable_tall_wall": {
            "object_center_m": [0.487, 0.410, 0.004],
            "obstacles_enabled": True,
            "box_id": "step_c_cable_tall_wall",
        },
    }
    for name, expected in expected_step_c.items():
        if profiles.get(name) != expected:
            failures.append(f"obstacles: profile {name} differs from Step C approval")
    expected_moving_wall = {
        "object_center_m": [0.487, 0.410, 0.004],
        "obstacles_enabled": True,
        "box_id": "step_c_cable_wall",
        "motion": {"axis": "x", "amplitude_m": 0.10, "period_s": 8.0},
    }
    if profiles.get("moving_wall") != expected_moving_wall:
        failures.append("obstacles: moving_wall profile differs from approval")
    expected_walls = {
        "step_c_cable_wall": ([0.487, 0.263, 0.08], [0.20, 0.05, 0.16]),
        "step_c_cable_tall_wall": ([0.487, 0.263, 0.30], [0.20, 0.05, 0.60]),
    }
    for name, (center, size) in expected_walls.items():
        box = boxes_by_id.get(name, {})
        if box.get("center_m") != center or box.get("size_m") != size:
            failures.append(f"obstacles: box {name} differs from Step C approval")
    print("[obstacles] schema and disabled-by-default policy checked")


def check_grasp_pass_policy():
    result = subprocess.run(
        [sys.executable, str(REPO / "scripts/test_candidate_contact_grasp_pass.py")],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        failures.append(
            "pass-policy: test_candidate_contact_grasp_pass.py failed: "
            + (result.stderr or result.stdout).strip().splitlines()[-1]
        )
    print("[pass-policy] obstacle-contact guard unit test run")


def check_step_c_result_extractor():
    result = subprocess.run(
        [sys.executable, str(REPO / "scripts/test_extract_step_c_result.py")],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        failures.append(
            "step-c-result: test_extract_step_c_result.py failed: "
            + (result.stderr or result.stdout).strip().splitlines()[-1]
        )
    print("[step-c-result] C2 planning-failure classifier unit test run")


def check_curobo_step_c_result_extractor():
    result = subprocess.run(
        [
            sys.executable,
            str(REPO / "scripts/test_extract_curobo_step_c_result.py"),
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        failures.append(
            "curobo-step-c-result: test_extract_curobo_step_c_result.py failed: "
            + (result.stderr or result.stdout).strip().splitlines()[-1]
        )
    print("[curobo-step-c-result] cuRobo/Newton C1 extractor unit test run")


def check_step_c_safe_pregrasp_policy():
    paths = (
        REPO / "scripts/run_step_c_pick_jazzy.sh",
        REPO / "scripts/run_step_c_plan_jazzy.sh",
    )
    forbidden = ("FEM_CABLE_SAFE_PREGRASP", "use_safe_pregrasp_path:=false")
    for path in paths:
        text = path.read_text(encoding="utf-8")
        for marker in forbidden:
            if marker in text:
                failures.append(
                    f"step-c-safe-route: {path.relative_to(REPO)} overrides {marker}"
                )
    print("[step-c-safe-route] Step C scripts preserve the verified safe route")


def check_moving_wall_pose():
    result = subprocess.run(
        [sys.executable, str(REPO / "scripts/test_moving_wall_pose.py")],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        failures.append(
            "moving-wall-pose: test_moving_wall_pose.py failed: "
            + (result.stderr or result.stdout).strip().splitlines()[-1]
        )
    print("[moving-wall-pose] sinusoidal pose unit test run")


def main():
    check("python", compile_python, tracked("*.py"))
    check("shell", check_shell, tracked("*.sh"))
    check("yaml", load_yaml, tracked("*.yaml", "*.yml"))
    check("json", load_json, tracked("*.json"))
    check("xml", parse_xml, tracked("*.xml", "*.xacro", "*.srdf", "*.urdf"))
    check_licenses()
    check_gripper_solver()
    check_cpp_calibration_table()
    check_obstacle_defaults()
    check_grasp_pass_policy()
    check_step_c_result_extractor()
    check_curobo_step_c_result_extractor()
    check_step_c_safe_pregrasp_policy()
    check_moving_wall_pose()
    if failures:
        print("\nFAILED:")
        for failure in failures:
            print(f"  - {failure}")
        sys.exit(1)
    print("\nAll checks passed.")


if __name__ == "__main__":
    main()
