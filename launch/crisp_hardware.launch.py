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

    system_config = PathJoinSubstitution(
        [FindPackageShare("pixi_kuka_ros2"), "config", "lbr_system_config.yaml"]
    )
    controllers = PathJoinSubstitution(
        [FindPackageShare("pixi_kuka_ros2"), "config", "controllers.yaml"]
    )
    hardware_overrides = PathJoinSubstitution(
        [FindPackageShare("pixi_kuka_ros2"), "config", "hardware_overrides.yaml"]
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
                " mode:=hardware",
                " system_config_path:=",
                system_config,
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
        parameters=[{"use_sim_time": False}, controllers, hardware_overrides],
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
            # The hardware interface blocks in an unbounded wait for the FRI
            # heartbeat, so the controller manager cannot complete a switch until
            # LBRServer has been started on the smartPAD. The spawner defaults
            # give up after five 20 s attempts, which is far less time than it
            # takes to walk to the pendant and answer four dialogs; when they run
            # out the spawner exits and no controller is ever loaded, including
            # fri_position_passthrough_controller. Wait instead of racing.
            "--controller-manager-timeout",
            "600",
            "--switch-timeout",
            "600",
            "joint_state_broadcaster",
            "estimated_wrench_interface",
            "lbr_state_broadcaster",
            "force_torque_broadcaster",
            # REST, not passthrough + zero effort. The trajectory controller activates holding
            # the MEASURED position (the LBR interface NaNs its commands on activation, so it
            # reads the state), and that fixed setpoint is what Sunrise's 200 Nm/rad joint
            # impedance holds the arm at. The passthrough mirrors the measured position instead,
            # so with it the arm is held by nothing: on 21 Sep 2026, with the UMI on and not
            # declared to Sunrise, the arm fell as soon as the stack came up.
            # scripts/kuka_crisp.py::safe_switch arms an overlay from here in one strict switch.
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
            "--controller-manager-timeout",
            "600",
            "--switch-timeout",
            "600",
            "--inactive",
            "cartesian_impedance_controller",
            "joint_impedance_controller",
            # Loaded inactive because it claims the same position command interfaces as
            # joint_trajectory_controller, which owns them at rest. It comes in together with
            # a torque overlay, in one strict swap; see scripts/kuka_crisp.py::safe_switch.
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
