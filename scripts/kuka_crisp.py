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
    """Arm cartesian_impedance_controller from REST, or go back to REST with
    zero_effort_controller.

    crisp_py's own switcher and Robot.home() are not for this cell: the switcher deactivates every
    controller that is not a broadcaster, with BEST_EFFORT, and home() moves in a fixed 5 s.
    """
    active = active_controllers(robot)
    if TRAJECTORY in active:
        if controller_name == ZERO_EFFORT:
            return
        if controller_name not in TORQUE_CONTROLLERS:
            raise RuntimeError(
                f"refusing to switch to {controller_name} from REST: only {CARTESIAN} or "
                f"{ZERO_EFFORT} can be switched to"
            )
        check_follows_target(controller_name)
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
        check_follows_target(controller_name)
        robot.controller_switcher_client.switch_controller(
            controller_name, controllers_that_should_be_active=HOLD_ACTIVE
        )


def check_follows_target(controller_name: str) -> None:
    """Raise for a torque controller other than the one that follows /lbr/target_pose."""
    if controller_name != CARTESIAN:
        raise RuntimeError(
            f"refusing to switch to {controller_name}: Sunrise's spring pulls toward the IK of "
            f"{NS}/target_pose, which only {CARTESIAN} follows"
        )


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
