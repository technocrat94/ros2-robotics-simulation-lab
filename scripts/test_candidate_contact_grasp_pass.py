#!/usr/bin/env python3
"""Dependency-free unit check for the endpoint's obstacle-contact pass guard."""

import ast
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
ENDPOINT = (
    REPO
    / "docs/experiments/newton-ros2-bridge/prototype"
    / "newton_moveit_grasp_endpoint.py"
)


def load_policy_function():
    tree = ast.parse(ENDPOINT.read_text(encoding="utf-8"), filename=str(ENDPOINT))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "candidate_contact_grasp_pass"
    )
    namespace = {}
    module = ast.Module(body=[function], type_ignores=[])
    exec(compile(module, str(ENDPOINT), "exec"), namespace)
    return namespace["candidate_contact_grasp_pass"]


def main():
    policy = load_policy_function()
    assert policy(True, 0) is True
    assert policy(True, 1) is False
    assert policy(True, 12) is False
    assert policy(False, 0) is False
    print("candidate_contact_grasp_pass obstacle guard: PASS")


if __name__ == "__main__":
    main()
