#!/usr/bin/env python3
"""GPU-free unit test for the Newton moving-wall pose function."""

import ast
import math
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
ENDPOINT = (
    REPO / "docs/experiments/newton-ros2-bridge/prototype/"
    "newton_moveit_grasp_endpoint.py"
)


def load_pose_function():
    tree = ast.parse(ENDPOINT.read_text(encoding="utf-8"), filename=str(ENDPOINT))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "moving_obstacle_center"
    )
    module = ast.Module(body=[function], type_ignores=[])
    namespace = {"math": math}
    exec(compile(module, str(ENDPOINT), "exec"), namespace)  # noqa: S102
    return namespace["moving_obstacle_center"]


def close(actual, expected):
    assert len(actual) == len(expected)
    assert all(abs(a - e) <= 1.0e-12 for a, e in zip(actual, expected)), (
        actual, expected
    )


def main():
    pose = load_pose_function()
    center = [0.487, 0.263, 0.08]
    motion = {"axis": "x", "amplitude_m": 0.10, "period_s": 8.0}
    close(pose(center, motion, 0.0), center)
    close(pose(center, motion, 2.0), [0.587, 0.263, 0.08])
    close(pose(center, motion, 4.0), center)
    close(pose(center, motion, 6.0), [0.387, 0.263, 0.08])
    phased = dict(motion, phase_rad=math.pi / 2.0)
    close(pose(center, phased, 0.0), [0.587, 0.263, 0.08])
    close(pose(center, phased, 2.0), center)
    close(pose(center, None, 123.0), center)
    try:
        pose(center, {"axis": "q", "amplitude_m": 0.1, "period_s": 8.0}, 0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("unsupported motion axis was accepted")
    print("moving_obstacle_center: PASS")


if __name__ == "__main__":
    main()
