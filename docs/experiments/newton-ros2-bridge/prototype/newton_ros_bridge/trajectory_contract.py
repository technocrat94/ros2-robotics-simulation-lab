"""Validation helpers shared by the ROS-to-Newton trajectory bridge."""

import math


ARM_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class TrajectoryValidationError(ValueError):
    """Raised when a trajectory is unsafe or incompatible with Newton."""


def validate_start_state(
    actual_by_name,
    newton_positions,
    max_error_rad=0.2,
):
    """Compare the ROS-reported arm state with Newton's initial state."""
    missing = [name for name in ARM_JOINT_NAMES if name not in actual_by_name]
    if missing:
        raise TrajectoryValidationError(
            "missing ROS joints: " + ", ".join(missing)
        )
    if len(newton_positions) != len(ARM_JOINT_NAMES):
        raise TrajectoryValidationError(
            f"Newton state has {len(newton_positions)} values; expected 6"
        )
    if not math.isfinite(max_error_rad) or max_error_rad < 0.0:
        raise TrajectoryValidationError("max_error_rad must be finite and non-negative")

    rows = []
    for index, name in enumerate(ARM_JOINT_NAMES):
        ros_position = float(actual_by_name[name])
        newton_position = float(newton_positions[index])
        if not math.isfinite(ros_position) or not math.isfinite(newton_position):
            raise TrajectoryValidationError(f"non-finite position for {name}")
        error = abs(ros_position - newton_position)
        rows.append(
            {
                "joint": name,
                "ros_rad": ros_position,
                "newton_rad": newton_position,
                "error_rad": error,
                "within_limit": error <= max_error_rad,
            }
        )

    mismatches = [row for row in rows if not row["within_limit"]]
    return {
        "pass": not mismatches,
        "threshold_rad": float(max_error_rad),
        "maximum_error_rad": max(row["error_rad"] for row in rows),
        "mismatches": mismatches,
        "joints": rows,
    }


def validate_trajectory(joint_names, points, actual_by_name, max_start_error_rad=0.2):
    """Validate names, samples, timing, and continuity of a trajectory."""
    names = [str(name) for name in joint_names]
    if len(names) != len(set(names)):
        raise TrajectoryValidationError("trajectory contains duplicate joint names")
    if set(names) != set(ARM_JOINT_NAMES):
        missing = sorted(set(ARM_JOINT_NAMES) - set(names))
        extra = sorted(set(names) - set(ARM_JOINT_NAMES))
        raise TrajectoryValidationError(
            f"joint set mismatch; missing={missing}, extra={extra}"
        )
    if not points:
        raise TrajectoryValidationError("trajectory contains no points")

    previous_time = -1.0
    normalized_points = []
    name_to_index = {name: index for index, name in enumerate(names)}
    for point_index, point in enumerate(points):
        positions = [float(value) for value in point["positions"]]
        if len(positions) != len(names):
            raise TrajectoryValidationError(
                f"point {point_index} has {len(positions)} positions; expected {len(names)}"
            )
        if not all(math.isfinite(value) for value in positions):
            raise TrajectoryValidationError(f"point {point_index} contains a non-finite position")
        time_s = float(point["time_s"])
        if not math.isfinite(time_s) or time_s < 0.0:
            raise TrajectoryValidationError(f"point {point_index} has invalid time")
        if time_s <= previous_time:
            raise TrajectoryValidationError(
                f"point {point_index} time is not strictly increasing"
            )
        previous_time = time_s
        normalized_points.append(
            {
                "time_s": time_s,
                "positions": [positions[name_to_index[name]] for name in ARM_JOINT_NAMES],
            }
        )

    start_check = validate_start_state(
        actual_by_name,
        normalized_points[0]["positions"],
        max_start_error_rad,
    )
    if not start_check["pass"]:
        names = ", ".join(row["joint"] for row in start_check["mismatches"])
        raise TrajectoryValidationError(
            f"trajectory start is discontinuous at: {names}"
        )

    return {
        "joint_names": list(ARM_JOINT_NAMES),
        "point_count": len(normalized_points),
        "duration_s": normalized_points[-1]["time_s"],
        "points": normalized_points,
        "start_check": start_check,
    }
