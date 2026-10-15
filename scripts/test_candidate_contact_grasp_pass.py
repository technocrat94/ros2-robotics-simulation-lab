#!/usr/bin/env python3
"""Dependency-free unit checks for endpoint acceptance state and metrics."""

import ast
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
ENDPOINT = (
    REPO
    / "docs/experiments/newton-ros2-bridge/prototype"
    / "newton_moveit_grasp_endpoint.py"
)


def load_policy_functions():
    tree = ast.parse(ENDPOINT.read_text(encoding="utf-8"), filename=str(ENDPOINT))
    names = {
        "candidate_contact_grasp_pass",
        "robot_obstacle_real_contact_count",
        "gripper_open_observed",
        "gripper_phase_flags",
    }
    functions = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    assert {function.name for function in functions} == names
    namespace = {}
    module = ast.Module(body=functions, type_ignores=[])
    exec(compile(module, str(ENDPOINT), "exec"), namespace)
    return namespace


def main():
    functions = load_policy_functions()
    policy = functions["candidate_contact_grasp_pass"]
    assert policy(True, 0) is True
    assert policy(True, 1) is False
    assert policy(True, 12) is False
    assert policy(False, 0) is False

    real_contact_count = functions["robot_obstacle_real_contact_count"]
    candidate = {
        "contact_force_magnitude_n": 0.0,
        "penetration_depth_m": 0.0,
    }
    loaded = {
        "contact_force_magnitude_n": 0.25,
        "penetration_depth_m": 0.0,
    }
    penetrating = {
        "contact_force_magnitude_n": 0.0,
        "penetration_depth_m": 0.001,
    }
    loaded_and_penetrating = {
        "contact_force_magnitude_n": 0.25,
        "penetration_depth_m": 0.001,
    }
    assert real_contact_count([candidate]) == 0
    assert real_contact_count([loaded]) == 1
    assert real_contact_count([penetrating]) == 1
    assert real_contact_count([loaded_and_penetrating]) == 1
    assert real_contact_count([candidate, loaded, penetrating]) == 2

    open_observed = functions["gripper_open_observed"]
    # Existing runs that start fully open still use the absolute threshold.
    assert open_observed(0.0, 0.0) is True
    assert open_observed(0.10, 0.0) is False
    # A bringup starting near 0.79 rad becomes open after a >=0.15 rad drop.
    assert open_observed(0.65, 0.79) is False
    assert open_observed(0.511, 0.79) is True

    phase_flags = functions["gripper_phase_flags"]

    def run_gripper_sequence(values):
        flags = (False, False, False)
        first = values[0]
        previous = first
        states = []
        for grip in values:
            flags = phase_flags(*flags, grip, previous, first)
            states.append(flags)
            previous = grip
        return states

    starts_open = run_gripper_sequence([0.0, 0.0, 0.40, 0.40, 0.0])
    assert starts_open[0] == (True, False, False)
    assert starts_open[2] == (True, True, False)
    assert starts_open[-1] == (True, True, True)

    starts_closed = run_gripper_sequence([
        0.79, 0.70, 0.63, 0.511, 0.511, 0.65, 0.79, 0.65, 0.511
    ])
    assert starts_closed[0] == (False, False, False)
    assert starts_closed[2] == (True, False, False)
    assert starts_closed[5] == (True, True, False)
    assert starts_closed[-1] == (True, True, True)

    print("endpoint acceptance metrics and open-state detection: PASS")


if __name__ == "__main__":
    main()
