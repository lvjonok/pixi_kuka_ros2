"""This cell from Python: the switch between REST and ARMED, a crisp_py Robot and the joint move."""

from dataclasses import dataclass, field

import numpy as np
from crisp_py.robot import Robot, RobotConfig
from lbr_cell import (
    ARMED,
    BASE_FRAME,
    CARTESIAN,
    HOME_DEGREES,
    JOINTS,
    NS,
    PASSTHROUGH,
    REST,
    TORQUE_CONTROLLERS,
    TRAJECTORY,
    ZERO_EFFORT,
    cell_state,
    controller_states,
    joint_limits,
    switch,
)
from rclpy.node import Node

# lbr_fri_ros2's command guard ends the FRI session when a measured joint velocity passes the
# description's limit.
HOME_SPEED_DEG_S = 10.0


@dataclass(kw_only=True)
class KukaIiwaConfig(RobotConfig):
    joint_names: list = field(default_factory=lambda: list(JOINTS))
    home_config: list = field(default_factory=lambda: list(np.deg2rad(HOME_DEGREES)))
    base_frame: str = BASE_FRAME
    target_frame: str = "lbr_link_ee"


def make_kuka_robot() -> Robot:
    """A crisp_py Robot. It publishes /lbr/target_pose at 50 Hz from the pose it first saw: with
    another publisher on the topic, crisp and the passthrough ignore every target."""
    return Robot(namespace=NS.strip("/"), robot_config=KukaIiwaConfig())


def active_controllers(robot: Robot) -> set[str]:
    return {name for name, state in controller_states(robot.node).items() if state == "active"}


def safe_switch(node: Node, controller_name: str) -> None:
    """Arm cartesian_impedance_controller from REST, or go to REST with zero_effort_controller,
    in one STRICT switch."""
    if controller_name not in (CARTESIAN, ZERO_EFFORT):
        raise RuntimeError(
            f"refusing to switch to {controller_name}: only {CARTESIAN} or {ZERO_EFFORT} can be "
            "switched to"
        )
    states = controller_states(node)
    active = {name for name, state in states.items() if state == "active"}
    state = cell_state(states)
    if PASSTHROUGH not in active and TRAJECTORY not in active:
        raise RuntimeError(
            f"refusing to switch: neither {PASSTHROUGH} nor {TRAJECTORY} is active, so nothing "
            f"owns the FRI position command. Active: {sorted(active)}"
        )
    if controller_name == ZERO_EFFORT:
        if state == "REST":
            print("already at REST: nothing to switch")
            return
        switch(
            node,
            activate=[name for name in REST if name not in active],
            deactivate=sorted({PASSTHROUGH, *TORQUE_CONTROLLERS} & active),
        )
    elif set(ARMED) <= active:
        print(f"already ARMED with {CARTESIAN}: nothing to switch")
    elif state != "REST":
        raise RuntimeError(
            f"refusing to arm: the cell is {state}. Go to REST first: --switch {ZERO_EFFORT}"
        )
    else:
        switch(node, activate=ARMED, deactivate=REST)


def move_to_home(
    robot: Robot, degrees: list[float] | None = None, speed_deg_s: float = HOME_SPEED_DEG_S
) -> None:
    """Move the arm to a joint configuration with the trajectory controller, at REST.

    MOVES THE ARM: in T1 with the enabling switch held and the workspace clear. Releasing the
    switch during the move ends the FRI session.
    """
    target = list(HOME_DEGREES if degrees is None else degrees)
    if len(target) != len(JOINTS):
        raise ValueError(f"expected {len(JOINTS)} joint angles, got {len(target)}")
    lower, upper, _ = joint_limits(robot.node)
    for joint, value, low, high in zip(JOINTS, target, np.degrees(lower), np.degrees(upper)):
        if not low <= value <= high:
            raise ValueError(
                f"{joint} target {value:.1f} deg is outside the description's limits "
                f"{low:.1f} to {high:.1f} deg"
            )

    state = cell_state(controller_states(robot.node))
    if state != "REST":
        raise RuntimeError(
            f"refusing to move: the cell is {state}. The joint move runs at REST, on {TRAJECTORY} "
            f"with {ZERO_EFFORT}"
        )

    start = np.rad2deg(np.asarray(robot.joint_values, dtype=float))
    delta = np.max(np.abs(np.asarray(target) - start))
    duration = max(delta / speed_deg_s, 2.0)
    print(
        f"largest joint delta {delta:.1f} deg, moving over {duration:.1f} s at {speed_deg_s} deg/s"
    )
    robot.joint_trajectory_controller_client.send_joint_config(
        robot.config.joint_names, list(np.deg2rad(target)), duration, blocking=True
    )
