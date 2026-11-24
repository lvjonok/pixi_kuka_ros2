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
    namespace = "lbr"
    mode = LaunchConfiguration("mode").perform(context)
    tool = LaunchConfiguration("tool").perform(context)
    hardware = mode == "hardware"
    umi = tool == "umi"
    config = [FindPackageShare("pixi_kuka_ros2"), "config"]

    if umi:
        robot_xacro = PathJoinSubstitution(
            [
                FindPackageShare("iris_robots_description"),
                "robots",
                "kuka_iiwa14",
                "iiwa14_umi.urdf.xacro",
            ]
        )
    else:
        robot_xacro = PathJoinSubstitution(
            [
                FindPackageShare("lbr_ros2_control"),
                "system_integration",
                "iiwa14",
                "iiwa14.xacro",
            ]
        )
    xacro = [FindExecutable(name="xacro"), " ", robot_xacro, " robot_name:=lbr", f" mode:={mode}"]
    if umi:
        xacro.append(" ros2_control:=true")
    if hardware:
        xacro += [
            " system_config_path:=",
            PathJoinSubstitution([*config, "lbr_system_config.yaml"]),
        ]
    # value_type=str: launch otherwise YAML-parses the document, and the UMI description's
    # comments do not survive that.
    robot_description = {"robot_description": ParameterValue(Command(xacro), value_type=str)}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    parameter_files = [PathJoinSubstitution([*config, "controllers.yaml"])]
    if umi:
        parameter_files.append(PathJoinSubstitution([*config, "controllers_umi.yaml"]))
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": False}, *parameter_files],
        remappings=[("~/robot_description", "robot_description")],
    )

    spawner = ["--controller-manager", "controller_manager"]
    fri_broadcasters = []
    if hardware:
        # The hardware interface blocks until the Sunrise application's FRI heartbeat, and a
        # spawner that gives up before it loads no controller.
        spawner += ["--controller-manager-timeout", "600", "--switch-timeout", "600"]
        fri_broadcasters = [
            "estimated_wrench_interface",
            "lbr_state_broadcaster",
            "force_torque_broadcaster",
        ]

    active_controllers = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        output="screen",
        arguments=[
            *spawner,
            "joint_state_broadcaster",
            *fri_broadcasters,
            "joint_trajectory_controller",
            "zero_effort_controller",
            "pose_broadcaster",
            "twist_broadcaster",
        ],
    )

    inactive_controllers = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        output="screen",
        arguments=[
            *spawner,
            "--inactive",
            "cartesian_impedance_controller",
            "fri_position_passthrough_controller",
        ],
    )

    nodes = [robot_state_publisher, controller_manager, active_controllers, inactive_controllers]
    if hardware:
        nodes.append(
            Node(
                package="pixi_kuka_ros2",
                executable="kuka_ext_torque_relay.py",
                namespace=namespace,
                output="screen",
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="hardware",
                choices=["hardware", "mock"],
                description="hardware: the cabinet over FRI. mock: mock_components, no cabinet.",
            ),
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
