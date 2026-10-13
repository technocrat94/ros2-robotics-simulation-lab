import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def bool_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    obstacle_config_path = (
        Path(get_package_share_directory("ur5_moveit_demo"))
        / "config" / "obstacles.yaml"
    )
    obstacle_config = json.loads(obstacle_config_path.read_text(encoding="utf-8"))
    if obstacle_config.get("schema_version") != 1 or len(obstacle_config.get("boxes", [])) != 1:
        raise RuntimeError("obstacles.yaml must use schema 1 with exactly one box")
    obstacle = obstacle_config["boxes"][0]
    center = obstacle["center_m"]
    size = obstacle["size_m"]
    arguments = [
        DeclareLaunchArgument("object_x", default_value="0.4869"),
        DeclareLaunchArgument("object_y", default_value="0.10915"),
        DeclareLaunchArgument("object_z", default_value="0.0100"),
        DeclareLaunchArgument("pregrasp_clearance", default_value="0.1500"),
        DeclareLaunchArgument("approach_distance", default_value="0.1000"),
        DeclareLaunchArgument("lift_distance", default_value="0.1200"),
        DeclareLaunchArgument("grasp_center_offset", default_value="0.1090"),
        DeclareLaunchArgument("closed_grip", default_value="0.3760"),
        DeclareLaunchArgument("use_width_calibration", default_value="false"),
        DeclareLaunchArgument("object_width_mm", default_value="50.0"),
        DeclareLaunchArgument("total_compression_mm", default_value="2.0"),
        DeclareLaunchArgument("uncompensated_approach_distance", default_value="0.1050"),
        DeclareLaunchArgument("velocity_scale", default_value="0.15"),
        DeclareLaunchArgument("lift_velocity_scale", default_value="0.03"),
        DeclareLaunchArgument("use_newton_object_pose", default_value="true"),
        DeclareLaunchArgument("pose_only", default_value="false"),
        DeclareLaunchArgument("plan_only", default_value="false"),
        DeclareLaunchArgument("use_safe_pregrasp_path", default_value="true"),
        DeclareLaunchArgument("safe_transit_margin", default_value="0.0500"),
        DeclareLaunchArgument("maximum_pregrasp_joint_travel", default_value="1.6000"),
        DeclareLaunchArgument("maximum_wrist_3_travel", default_value="3.2500"),
        DeclareLaunchArgument("pregrasp_only", default_value="false"),
        DeclareLaunchArgument("skip_pregrasp_motion", default_value="false"),
        DeclareLaunchArgument("use_named_start", default_value="false"),
        DeclareLaunchArgument("approach_plan_only", default_value="false"),
        DeclareLaunchArgument("approach_only", default_value="false"),
        DeclareLaunchArgument("lift_only", default_value="false"),
        DeclareLaunchArgument("grasp_roll_deg", default_value="180.0"),
        DeclareLaunchArgument("grasp_pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("grasp_yaw_deg", default_value="90.0"),
        DeclareLaunchArgument("floor_z", default_value="0.0"),
        DeclareLaunchArgument("floor_size", default_value="3.0"),
        DeclareLaunchArgument("floor_thickness", default_value="0.02"),
        DeclareLaunchArgument("object_pose_topic", default_value="/newton/object_pose"),
        DeclareLaunchArgument("object_pose_timeout", default_value="5.0"),
        DeclareLaunchArgument(
            "obstacles_enabled",
            default_value=str(bool(obstacle_config["enabled"])).lower(),
        ),
        DeclareLaunchArgument("obstacle_id", default_value=obstacle["id"]),
        DeclareLaunchArgument("obstacle_frame", default_value=obstacle["frame_id"]),
        DeclareLaunchArgument("obstacle_x", default_value=str(center[0])),
        DeclareLaunchArgument("obstacle_y", default_value=str(center[1])),
        DeclareLaunchArgument("obstacle_z", default_value=str(center[2])),
        DeclareLaunchArgument("obstacle_size_x", default_value=str(size[0])),
        DeclareLaunchArgument("obstacle_size_y", default_value=str(size[1])),
        DeclareLaunchArgument("obstacle_size_z", default_value=str(size[2])),
    ]

    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur5_moveit_demo")
        .robot_description(
            file_path="urdf/ur5_robotiq.urdf.xacro",
            mappings={
                "name": "ur",
                "ur_type": "ur5",
                "use_fake_hardware": "true",
            },
        )
        .robot_description_semantic(
            file_path="srdf/ur5_robotiq.srdf.xacro",
            mappings={"name": "ur", "prefix": ""},
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
            load_all=False,
        )
        .to_moveit_configs()
    )

    parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {
            name: float_parameter(name)
            for name in (
                "object_x", "object_y", "object_z", "pregrasp_clearance",
                "approach_distance",
                "lift_distance", "grasp_center_offset", "closed_grip",
                "object_width_mm", "total_compression_mm",
                "uncompensated_approach_distance",
                "velocity_scale", "lift_velocity_scale", "object_pose_timeout",
                "grasp_roll_deg", "grasp_pitch_deg", "grasp_yaw_deg",
                "floor_z", "floor_size", "floor_thickness",
                "safe_transit_margin", "maximum_pregrasp_joint_travel",
                "maximum_wrist_3_travel",
                "obstacle_x", "obstacle_y", "obstacle_z",
                "obstacle_size_x", "obstacle_size_y", "obstacle_size_z",
            )
        },
        {"use_newton_object_pose": bool_parameter("use_newton_object_pose")},
        {"pose_only": bool_parameter("pose_only")},
        {"plan_only": bool_parameter("plan_only")},
        {"use_safe_pregrasp_path": bool_parameter("use_safe_pregrasp_path")},
        {"pregrasp_only": bool_parameter("pregrasp_only")},
        {"skip_pregrasp_motion": bool_parameter("skip_pregrasp_motion")},
        {"use_named_start": bool_parameter("use_named_start")},
        {"approach_plan_only": bool_parameter("approach_plan_only")},
        {"approach_only": bool_parameter("approach_only")},
        {"lift_only": bool_parameter("lift_only")},
        {"use_width_calibration": bool_parameter("use_width_calibration")},
        {"obstacles_enabled": bool_parameter("obstacles_enabled")},
        {"object_pose_topic": LaunchConfiguration("object_pose_topic")},
        {"obstacle_id": LaunchConfiguration("obstacle_id")},
        {"obstacle_frame": LaunchConfiguration("obstacle_frame")},
    ]

    return LaunchDescription(arguments + [
        Node(
            package="ur5_moveit_demo",
            executable="pick_at_position",
            name="pick_at_position",
            output="screen",
            parameters=parameters,
        )
    ])
