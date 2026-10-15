#!/usr/bin/env python3
"""Dependency-free checks for Step C result-log classification."""

from extract_step_c_result import c2_planning_failed


def main():
    assert c2_planning_failed("Safe pre-grasp path is incomplete") is True
    assert c2_planning_failed("Absolute pre-grasp planning failed") is True
    assert c2_planning_failed("No motion plan found") is True
    assert c2_planning_failed("PLANNING_FAILED") is True
    assert c2_planning_failed("PLAN_ONLY_SUCCEEDED") is False
    print("Step C C2 planning-failure classification: PASS")


if __name__ == "__main__":
    main()
