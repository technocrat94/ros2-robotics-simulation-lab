from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    package_share = FindPackageShare("ur5_moveit_demo")

    controllers_file = PathJoinSubstitution([
        package_share,
        "config",
        "combined_controllers.yaml",
    ])

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
        .trajectory_execution(
            file_path="config/controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
            load_all=False,
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    robot_description = moveit_config.robot_description

    # 發布 UR5＋Robotiq 的完整 TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 唯一一個 Controller Manager，同時管理 UR5 與 Robotiq
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controllers_file,
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robotiq_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "robotiq_gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robotiq_activation_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "robotiq_activation_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    rviz_config = PathJoinSubstitution([
        package_share,
        "rviz",
        "view_robot.rviz",
    ])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,

        TimerAction(
            period=2.0,
            actions=[joint_state_broadcaster],
        ),

        TimerAction(
            period=3.0,
            actions=[joint_trajectory_controller],
        ),

        TimerAction(
            period=3.5,
            actions=[robotiq_gripper_controller],
        ),

        TimerAction(
            period=4.0,
            actions=[robotiq_activation_controller],
        ),

        TimerAction(
            period=5.0,
            actions=[move_group, rviz],
        ),
    ])
