from launch import LaunchDescription

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

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
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
            load_all=False,
        )
        .to_moveit_configs()
    )

    move_xyz_node = Node(
        package="ur5_moveit_demo",
        executable="move_xyz",
        name="move_xyz",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        move_xyz_node,
    ])
