from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)


def generate_launch_description():

    # 與 move_group 使用完全相同的 UR5＋Robotiq 模型
    combined_xacro = PathJoinSubstitution([
        FindPackageShare("ur5_moveit_demo"),
        "urdf",
        "ur5_robotiq.urdf.xacro",
    ])

    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        combined_xacro,
        " ",
        "name:=ur",
        " ",
        "ur_type:=ur5",
        " ",
        "use_fake_hardware:=true",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    # 專案 SRDF；沿用 UR 的 ur_manipulator 並加入 Robotiq 碰撞設定
    robot_description_semantic_content = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur5_moveit_demo"),
            "srdf",
            "ur5_robotiq.srdf.xacro",
        ]),
        " ",
        "name:=ur",
        " ",
        'prefix:=""',
    ])

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content,
            value_type=str,
        )
    }

    robot_description_kinematics = PathJoinSubstitution([
        FindPackageShare("ur5_moveit_demo"),
        "config",
        "kinematics.yaml",
    ])

    move_xyz_node = Node(
        package="ur5_moveit_demo",
        executable="move_xyz",
        name="move_xyz",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        move_xyz_node,
    ])
