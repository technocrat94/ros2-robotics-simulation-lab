from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_share = FindPackageShare("ur5_moveit_demo")

    xacro_file = PathJoinSubstitution([
        package_share,
        "urdf",
        "ur5_robotiq.urdf.xacro",
    ])

    controllers_file = PathJoinSubstitution([
        package_share,
        "config",
        "combined_controllers.yaml",
    ])

    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        xacro_file,
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

    # 官方 MoveIt，但改用我們的 UR5＋Robotiq 組合模型
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": "ur5",
            "description_package": "ur5_moveit_demo",
            "description_file": "ur5_robotiq.urdf.xacro",
            "moveit_config_package": "ur5_moveit_demo",
            "moveit_config_file": "ur5_robotiq.srdf.xacro",
            "launch_rviz": "true",
            "use_sim_time": "true",
        }.items(),
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
            actions=[moveit],
        ),
    ])
