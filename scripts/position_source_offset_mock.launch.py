# ruff: noqa: I001
"""The mock stack with mock_components' position_state_following_offset.

For `check-mock --offset-lagged`. GenericSystem then reports each position state as the last
command plus `offset`, so the measured position steps by `offset` off the passthrough's output
every update. Every mode drifts at offset x 500 rad/s on it: run nothing else against it. Refuses
to start outside domain 77 on the loopback CycloneDDS config.
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
        PLUGIN, f'{PLUGIN}<param name="position_state_following_offset">{offset!r}</param>'
    )

    def node(package: str, executable: str, **more) -> Node:
        return Node(
            package=package, executable=executable, namespace="lbr", output="screen", **more
        )

    spawner = ["--controller-manager", "controller_manager"]
    return [
        node(
            "robot_state_publisher",
            "robot_state_publisher",
            parameters=[{"robot_description": urdf, "use_sim_time": False}],
        ),
        node(
            "controller_manager",
            "ros2_control_node",
            parameters=[{"use_sim_time": False}, f"{share}/config/controllers.yaml"],
            remappings=[("~/robot_description", "robot_description")],
        ),
        node(
            "controller_manager",
            "spawner",
            arguments=[
                *spawner,
                "joint_state_broadcaster",
                "joint_trajectory_controller",
                "zero_effort_controller",
                "pose_broadcaster",
                "twist_broadcaster",
            ],
        ),
        node(
            "controller_manager",
            "spawner",
            arguments=[
                *spawner,
                "--inactive",
                "cartesian_impedance_controller",
                "joint_impedance_controller",
                "fri_position_passthrough_controller",
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [DeclareLaunchArgument("offset", default_value="0.0001"), OpaqueFunction(function=_nodes)]
    )
