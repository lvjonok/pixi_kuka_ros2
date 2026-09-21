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

    system_config = PathJoinSubstitution(
        [FindPackageShare("pixi_kuka_ros2"), "config", "lbr_system_config.yaml"]
    )
    controllers = PathJoinSubstitution(
        [FindPackageShare("pixi_kuka_ros2"), "config", "controllers.yaml"]
    )
    controllers_umi = PathJoinSubstitution(
        [FindPackageShare("pixi_kuka_ros2"), "config", "controllers_umi.yaml"]
    )
    hardware_overrides = PathJoinSubstitution(
        [FindPackageShare("pixi_kuka_ros2"), "config", "hardware_overrides.yaml"]
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
                    " mode:=hardware",
                    tool_args,
                    " system_config_path:=",
                    system_config,
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
            hardware_overrides,
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

    # KUKA's external-torque estimate as a plain JointState (<ns>/kuka_external_torque), for
    # recorders without lbr_fri_idl. Reads lbr_state from lbr_state_broadcaster above.
    ext_torque_relay = Node(
        package="pixi_kuka_ros2",
        executable="kuka_ext_torque_relay.py",
        namespace=namespace,
        output="screen",
    )

    return [
        robot_state_publisher,
        controller_manager,
        active_controllers,
        inactive_crisp_controllers,
        ext_torque_relay,
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
