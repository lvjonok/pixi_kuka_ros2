"""crisp_py on this cell: the robot, the switch between REST and ARMED, and the joint move."""

from dataclasses import dataclass, field

import numpy as np
from crisp_py.robot import Robot, RobotConfig
from lbr_cell import (
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
    controller_states,
    joint_limits,
    switch,
)
from position_source import running, source

# lbr_fri_ros2's command guard ends the FRI session when a measured joint velocity passes the
# description's limit.
HOME_SPEED_DEG_S = 10.0

# crisp_py's switcher deactivates every active controller whose name does not end in
# "broadcaster", unless it is named here.
HOLD_ACTIVE = [PASSTHROUGH, "estimated_wrench_interface"]


@dataclass(kw_only=True)
class KukaIiwaConfig(RobotConfig):
    joint_names: list = field(default_factory=lambda: list(JOINTS))
    home_config: list = field(default_factory=lambda: list(np.deg2rad(HOME_DEGREES)))
    base_frame: str = BASE_FRAME
    target_frame: str = "lbr_link_ee"


def make_kuka_robot() -> Robot:
    return Robot(namespace=NS.strip("/"), robot_config=KukaIiwaConfig())


def active_controllers(robot: Robot) -> set[str]:
    return {name for name, state in controller_states(robot.node).items() if state == "active"}


def safe_switch(robot: Robot, controller_name: str) -> None:
    """Arm a torque controller from REST, go back to REST with zero_effort_controller, or change
    the torque controller while ARMED.

    crisp_py's own switcher and Robot.home() are not for this cell: the switcher deactivates every
    controller that is not a broadcaster, with BEST_EFFORT, and home() moves in a fixed 5 s.
    """
    active = active_controllers(robot)
    if TRAJECTORY in active:
        if controller_name == ZERO_EFFORT:
            return
        if controller_name not in TORQUE_CONTROLLERS:
            raise RuntimeError(
                f"refusing to switch to {controller_name} from REST: only a torque controller "
                f"({', '.join(TORQUE_CONTROLLERS)}) or {ZERO_EFFORT} can be switched to"
            )
        check_position_source(robot, controller_name, armed=False)
        switch(robot.node, activate=(PASSTHROUGH, controller_name), deactivate=REST)
    elif PASSTHROUGH not in active:
        raise RuntimeError(
            f"refusing to switch: neither {PASSTHROUGH} nor {TRAJECTORY} is active, so nothing "
            f"owns the FRI position command. Active: {sorted(active)}"
        )
    elif controller_name == ZERO_EFFORT:
        overlays = sorted(set(TORQUE_CONTROLLERS) & active)
        switch(robot.node, activate=REST, deactivate=(PASSTHROUGH, *overlays))
    else:
        check_position_source(robot, controller_name, armed=True)
        robot.controller_switcher_client.switch_controller(
            controller_name, controllers_that_should_be_active=HOLD_ACTIVE
        )


def check_position_source(robot: Robot, controller_name: str, armed: bool) -> None:
    """Raise unless the torque controller may run over the passthrough's position_source.

    Under ik_target Sunrise's spring pulls toward the IK of /lbr/target_pose, which only
    cartesian_impedance_controller follows.
    """
    if controller_name == CARTESIAN:
        return
    why = (
        f"{controller_name} does not track {NS}/target_pose, and only {CARTESIAN} may run over a "
        "passthrough that is not in `measured`. Go to REST and `position-source set measured` first"
    )
    refusing = f"refusing to switch to {controller_name}: {PASSTHROUGH}"
    parameter = source(robot.node)
    if parameter != "measured":
        raise RuntimeError(f"{refusing} position_source is {parameter}. {why}")
    if armed:
        state = running(robot.node, 2.0)
        mode = state["mode"] if state else "an unknown mode (no ~/state)"
        if mode != "measured":
            raise RuntimeError(f"{refusing} is running {mode}. {why}")


def move_to_home(
    robot: Robot, degrees: list[float] | None = None, speed_deg_s: float = HOME_SPEED_DEG_S
) -> None:
    """Move the arm to a joint configuration with the trajectory controller, and leave it at REST.

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

    active = active_controllers(robot)
    if PASSTHROUGH not in active and TRAJECTORY not in active:
        raise RuntimeError(
            f"refusing to move: neither {PASSTHROUGH} nor {TRAJECTORY} is active, so nothing owns "
            f"the FRI position command. Active: {sorted(active)}"
        )
    if ZERO_EFFORT not in active:
        raise RuntimeError(
            f"refusing to move: {ZERO_EFFORT} is not active. The torque overlay must be zero for "
            f"the whole move. Active: {sorted(active)}"
        )
    overlays = sorted(set(TORQUE_CONTROLLERS) & active)
    if overlays:
        raise RuntimeError(
            f"refusing to move: torque controllers are active: {overlays}. Switch to {ZERO_EFFORT} "
            "first"
        )

    start = np.rad2deg(np.asarray(robot.joint_values, dtype=float))
    delta = np.max(np.abs(np.asarray(target) - start))
    duration = max(delta / speed_deg_s, 2.0)
    print(
        f"largest joint delta {delta:.1f} deg, moving over {duration:.1f} s at {speed_deg_s} deg/s"
    )

    if TRAJECTORY not in active:
        switch(robot.node, activate=[TRAJECTORY], deactivate=[PASSTHROUGH])
    robot.joint_trajectory_controller_client.send_joint_config(
        robot.config.joint_names, list(np.deg2rad(target)), duration, blocking=True
    )
