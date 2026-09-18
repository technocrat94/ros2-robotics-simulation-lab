from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("object_x", default_value="0.4869"),
        DeclareLaunchArgument("object_y", default_value="0.10915"),
        DeclareLaunchArgument("object_z", default_value="0.0100"),
        DeclareLaunchArgument("pregrasp_clearance", default_value="0.1500"),
        DeclareLaunchArgument("lift_distance", default_value="0.1200"),
        DeclareLaunchArgument("grasp_center_offset", default_value="0.1090"),
        DeclareLaunchArgument("closed_grip", default_value="0.3760"),
        DeclareLaunchArgument("velocity_scale", default_value="0.15"),
    ]

    combined_xacro = PathJoinSubstitution([
        FindPackageShare("ur5_moveit_demo"), "urdf", "ur5_robotiq.urdf.xacro"
    ])
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ", combined_xacro,
                " ", "name:=ur", " ", "ur_type:=ur5",
                " ", "use_fake_hardware:=true",
            ]),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([
                    FindPackageShare("ur5_moveit_demo"),
                    "srdf", "ur5_robotiq.srdf.xacro",
                ]),
                " ", "name:=ur", " ", 'prefix:=""',
            ]),
            value_type=str,
        )
    }

    parameters = [
        robot_description,
        robot_description_semantic,
        PathJoinSubstitution([
            FindPackageShare("ur5_moveit_demo"), "config", "kinematics.yaml"
        ]),
        {
            name: float_parameter(name)
            for name in (
                "object_x", "object_y", "object_z", "pregrasp_clearance",
                "lift_distance", "grasp_center_offset", "closed_grip",
                "velocity_scale",
            )
        },
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
