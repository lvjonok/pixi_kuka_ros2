# ruff: noqa: I001
"""crisp_mock.launch.py (tool:=none) with mock_components' position_state_following_offset.

For scripts/position_source_mock_check.py --offset-lagged only. GenericSystem then reports each
position state as the last command plus `offset`, so the measured position steps by `offset`
off the passthrough's output every update: the perturbation `lagged` needs, which the stock mock
(state = command) never makes. Every mode drifts at offset x 500 rad/s here; run nothing else
against it.

    ROS_DOMAIN_ID=77 CYCLONEDDS_URI=file://$PWD/config/cyclonedds_loopback.xml \
        pixi run -e jazzy ros2 launch scripts/position_source_offset_mock.launch.py offset:=0.0001

Refuses to start anything outside domain 77 on the loopback CycloneDDS config, the same test as
position_source_mock_check.py: it brings up a /lbr/controller_manager, and the workstation shares
a LAN with the cell.
"""

import subprocess
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PLUGIN = "<plugin>mock_components/GenericSystem</plugin>"


def _nodes(context) -> list:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from position_source_mock_check import environment_problem

    problem = environment_problem()
    if problem:
        raise RuntimeError(f"{problem}: the offset mock runs on the loopback domain 77 only")
    namespace = LaunchConfiguration("namespace")
    offset = float(LaunchConfiguration("offset").perform(context))
    share = FindPackageShare("pixi_kuka_ros2").perform(context)
    xacro = FindPackageShare("lbr_ros2_control").perform(context)
    urdf = subprocess.run(
        [
            "xacro",
            f"{xacro}/system_integration/iiwa14/iiwa14.xacro",
            "robot_name:=lbr",
            "mode:=mock",
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    if urdf.count(PLUGIN) != 1:
        raise RuntimeError(f"expected one {PLUGIN} in the description")
    urdf = urdf.replace(
        PLUGIN,
        f'{PLUGIN}<param name="position_state_following_offset">{offset!r}</param>',
    )
    robot_description = {"robot_description": urdf}

    spawner = {"package": "controller_manager", "executable": "spawner", "namespace": namespace}
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[robot_description, {"use_sim_time": False}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace,
            output="screen",
            parameters=[{"use_sim_time": False}, f"{share}/config/controllers.yaml"],
            remappings=[("~/robot_description", "robot_description")],
        ),
        Node(
            **spawner,
            output="screen",
            arguments=[
                "--controller-manager",
                "controller_manager",
                "joint_state_broadcaster",
                "joint_trajectory_controller",
                "zero_effort_controller",
                "pose_broadcaster",
                "twist_broadcaster",
            ],
        ),
        Node(
            **spawner,
            output="screen",
            arguments=[
                "--controller-manager",
                "controller_manager",
                "--inactive",
                "cartesian_impedance_controller",
                "joint_impedance_controller",
                "fri_position_passthrough_controller",
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="lbr"),
            DeclareLaunchArgument("offset", default_value="0.0001"),
            OpaqueFunction(function=_nodes),
        ]
    )
