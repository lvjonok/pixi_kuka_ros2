# ruff: noqa: I001

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")

    controllers = PathJoinSubstitution(
        [FindPackageShare("pixi_kuka_ros2"), "config", "controllers.yaml"]
    )
    robot_xacro = PathJoinSubstitution(
        [
            FindPackageShare("lbr_ros2_control"),
            "system_integration",
            "iiwa14",
            "iiwa14.xacro",
        ]
    )
    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                robot_xacro,
                " robot_name:=lbr",
                " mode:=mock",
            ]
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": False}, controllers],
        remappings=[("~/robot_description", "robot_description")],
    )

    active_controllers = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        output="screen",
        arguments=[
            "--controller-manager",
            "controller_manager",
            "joint_state_broadcaster",
            # Same REST as crisp_hardware.launch.py: trajectory + zero effort.
            "joint_trajectory_controller",
            "zero_effort_controller",
            "pose_broadcaster",
            "twist_broadcaster",
        ],
    )

    inactive_crisp_controllers = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        output="screen",
        arguments=[
            "--controller-manager",
            "controller_manager",
            "--inactive",
            "cartesian_impedance_controller",
            "joint_impedance_controller",
            "fri_position_passthrough_controller",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="lbr"),
            robot_state_publisher,
            controller_manager,
            active_controllers,
            inactive_crisp_controllers,
        ]
    )
