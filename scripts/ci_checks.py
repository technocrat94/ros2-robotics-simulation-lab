#!/usr/bin/env python3
"""GPU-free repository checks run by GitHub Actions.

These checks do not start ROS, MoveIt, or Newton. They catch broken syntax,
malformed configuration, license drift, and regressions in the gripper-width
calibration that the verified FEM ground pick depends on.
"""

import json
import py_compile
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


def main():
    check("python", compile_python, tracked("*.py"))
    check("shell", check_shell, tracked("*.sh"))
    check("yaml", load_yaml, tracked("*.yaml", "*.yml"))
    check("json", load_json, tracked("*.json"))
    check("xml", parse_xml, tracked("*.xml", "*.xacro", "*.srdf", "*.urdf"))
    check_licenses()
    check_gripper_solver()
    if failures:
        print("\nFAILED:")
        for failure in failures:
            print(f"  - {failure}")
        sys.exit(1)
    print("\nAll checks passed.")


if __name__ == "__main__":
    main()
