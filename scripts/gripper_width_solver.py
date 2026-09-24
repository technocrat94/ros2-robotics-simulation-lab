#!/usr/bin/env python3
"""Solve gripper command and closure-height compensation from object width."""

import argparse
import csv
import json
from pathlib import Path


DEFAULT_TABLE = Path.home() / "ur5_ws/gripper_kinematics_calibration.csv"


def interpolate(target_gap, rows):
    # Gap decreases as the command angle increases.
    for lower, upper in zip(rows, rows[1:]):
        gap_a = lower["estimated_pad_gap_m"]
        gap_b = upper["estimated_pad_gap_m"]
        if gap_a >= target_gap >= gap_b:
            fraction = (gap_a - target_gap) / (gap_a - gap_b)
            command = lower["command_rad"] + fraction * (
                upper["command_rad"] - lower["command_rad"]
            )
            drop = lower["closure_drop_m"] + fraction * (
                upper["closure_drop_m"] - lower["closure_drop_m"]
            )
            return command, drop, lower["command_rad"], upper["command_rad"]
    minimum = rows[-1]["estimated_pad_gap_m"]
    maximum = rows[0]["estimated_pad_gap_m"]
    raise ValueError(
        f"target gap {target_gap * 1000:.2f} mm is outside the calibrated "
        f"range {minimum * 1000:.2f}--{maximum * 1000:.2f} mm"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--object-width-mm", type=float, required=True)
    parser.add_argument(
        "--total-compression-mm",
        type=float,
        default=0.0,
        help="Total width reduction across both sides of a soft object",
    )
    parser.add_argument("--table", type=Path, default=DEFAULT_TABLE)
    args = parser.parse_args()

    target_gap_mm = args.object_width_mm - args.total_compression_mm
    if target_gap_mm <= 0.0:
        raise SystemExit("Target gap must remain positive")

    with args.table.open(newline="", encoding="utf-8") as stream:
        rows = [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]

    try:
        command, drop, bracket_low, bracket_high = interpolate(
            target_gap_mm / 1000.0, rows
        )
    except ValueError as error:
        raise SystemExit(str(error)) from error

    result = {
        "object_width_mm": args.object_width_mm,
        "total_compression_mm": args.total_compression_mm,
        "target_pad_gap_mm": target_gap_mm,
        "gripper_command_rad": command,
        "closure_drop_mm": drop * 1000.0,
        "raise_open_pose_by_mm": drop * 1000.0,
        "interpolation_bracket_rad": [bracket_low, bracket_high],
    }
    print("GRIPPER_SOLUTION", json.dumps(result, separators=(",", ":")))
    print(
        "NOTE estimated pad gap uses the 85 mm nominal opening and must be "
        "validated with Newton contact feedback"
    )


if __name__ == "__main__":
    main()
