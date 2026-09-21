# ruff: noqa: I001

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _nodes(context) -> list:
    namespace = LaunchConfiguration("namespace")

    controllers = PathJoinSubstitution(
        [FindPackageShare("pixi_kuka_ros2"), "config", "controllers.yaml"]
    )
    controllers_umi = PathJoinSubstitution(
        [FindPackageShare("pixi_kuka_ros2"), "config", "controllers_umi.yaml"]
    )
    # tool:=none is upstream's bare iiwa14, the description every excitation and teleop session
    # so far ran on. tool:=umi is the same arm and the same ros2_control block (byte-identical,
    # checked 21 Sep 2026) plus the UMI, and it moves the Cartesian endpoint to the camera --
    # see config/controllers_umi.yaml for who must NOT run against that.
    tool = LaunchConfiguration("tool").perform(context)
    if tool == "umi":
        robot_xacro = PathJoinSubstitution(
            [
                FindPackageShare("iris_robots_description"),
                "robots",
                "kuka_iiwa14",
                "iiwa14_umi.urdf.xacro",
            ]
        )
        tool_args = " ros2_control:=true"
    else:
        robot_xacro = PathJoinSubstitution(
            [
                FindPackageShare("lbr_ros2_control"),
                "system_integration",
                "iiwa14",
                "iiwa14.xacro",
            ]
        )
        tool_args = ""
    robot_description = {
        # str, explicitly: launch otherwise YAML-parses the document, and the UMI description's
        # comments do not survive that.
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    robot_xacro,
                    " robot_name:=lbr",
                    " mode:=mock",
                    tool_args,
                ]
            ),
            value_type=str,
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
        parameters=[
            {"use_sim_time": False},
            controllers,
            *([controllers_umi] if tool == "umi" else []),
        ],
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

    return [
        robot_state_publisher,
        controller_manager,
        active_controllers,
        inactive_crisp_controllers,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="lbr"),
            DeclareLaunchArgument(
                "tool",
                default_value="none",
                choices=["none", "umi"],
                description="none: bare flange, endpoint lbr_link_ee. umi: UMI fitted, "
                "endpoint lbr_umi_camera.",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
