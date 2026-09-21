"""crisp_py bindings for this KUKA iiwa14 workspace.

crisp_py's stock IiwaConfig does not match this cell, in two ways that matter:

1. Its home_config commands A4 = -3*pi/4 (-135 deg). The iiwa14 R820 A4 limit is
   +/-120 deg, so that pose is unreachable. HOME_DEGREES below replaces it.
2. Its base_frame is "world", which this description does not publish. The base
   link is lbr_link_0.

Separately, crisp_py's ControllerSwitcherClient deactivates every active
controller whose name does not end in "broadcaster" unless it is named in
controllers_that_should_be_active. On this robot that would deactivate
fri_position_passthrough_controller, which must stay active for as long as any
CRISP controller is writing a torque overlay: it is what pins the FRI position
setpoint to the measured position. Use safe_switch below rather than calling
robot.controller_switcher_client.switch_controller directly.

It also issues every switch with strictness BEST_EFFORT, which reports success
after a partial switch. move_to_home below uses strict_switch instead, which
sends STRICT and therefore fails as a whole or not at all.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
from controller_manager_msgs.srv import SwitchController
from crisp_py.robot import Robot, RobotConfig

# Controllers that must remain active across every switch.
#
# fri_position_passthrough_controller holds the FRI position setpoint at the
# measured position. estimated_wrench_interface is the sensor source that
# force_torque_broadcaster reads, and does not end in "broadcaster", so the
# stock switcher would drop it.
HOLD_ACTIVE = [
    "fri_position_passthrough_controller",
    "estimated_wrench_interface",
]

# Degrees. Chosen for limit margin and conditioning, not copied from another arm:
# every joint sits at least 45 deg from its limit, and manipulability
# sqrt(det(J J^T)) is 0.126 against 0.044 for crisp_py's pose and exactly 0 at
# the all-zeros candle pose, which is fully singular.
HOME_DEGREES = [0.0, 30.0, 0.0, -75.0, 0.0, 75.0, 0.0]

PASSTHROUGH = "fri_position_passthrough_controller"
TRAJECTORY = "joint_trajectory_controller"
ZERO_EFFORT = "zero_effort_controller"

# Controllers that write a torque overlay. None of them may be active during a
# trajectory move: the move commands positions, and a simultaneous overlay would
# fight the Sunrise impedance controller that is tracking them.
TORQUE_CONTROLLERS = ["cartesian_impedance_controller", "joint_impedance_controller"]

# iiwa14 R820 axis limits in degrees, from the KUKA specification. Checked
# locally so an out-of-range target is refused here rather than by the FRI
# command guard, which reacts by dropping the session.
JOINT_LIMITS_DEG = [170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0]

# Degrees per second, per joint, used to derive the trajectory duration. Well
# under the axis velocity limits: the FRI command guard trips on *measured*
# velocity exceeding the URDF limit and answers by stopping the session, and
# these moves are commissioning moves run in T1 with a hand on the enabling
# switch. Slow is the whole point.
HOME_SPEED_DEG_S = 10.0


@dataclass(kw_only=True)
class KukaIiwaConfig(RobotConfig):
    """RobotConfig for the lbr-stack iiwa14 bringup in this workspace."""

    joint_names: list = field(
        default_factory=lambda: [
            "lbr_A1",
            "lbr_A2",
            "lbr_A3",
            "lbr_A4",
            "lbr_A5",
            "lbr_A6",
            "lbr_A7",
        ]
    )
    home_config: list = field(default_factory=lambda: list(np.deg2rad(HOME_DEGREES)))
    base_frame: str = "lbr_link_0"
    target_frame: str = "lbr_link_ee"


def make_kuka_robot(namespace: str = "lbr", **kwargs) -> Robot:
    """Build a crisp_py Robot wired to this workspace's namespace and frames."""
    return Robot(namespace=namespace, robot_config=KukaIiwaConfig(), **kwargs)


def safe_switch(robot: Robot, controller_name: str) -> bool | None:
    """Switch controllers without ever leaving the arm unheld.

    Two resting pairs exist, and a switch between them is ONE strict switch:

    * REST: joint_trajectory_controller + zero_effort_controller. The trajectory controller
      holds a FIXED position setpoint, so Sunrise's 200 Nm/rad joint impedance carries the
      arm. This is what the stack starts in (launch/crisp_hardware.launch.py) and where
      --home leaves it.
    * ARMED: fri_position_passthrough_controller + a torque overlay (the CRISP controllers).
      The passthrough mirrors the measured position, so Sunrise contributes nothing and the
      overlay holds the arm.

    passthrough + zero_effort -- the old resting state -- holds NOTHING: Sunrise's spring is
    centred on where the arm already is. The bare arm sagged in it (17 Sep 2026); with the UMI
    on and undeclared, 21 Sep 2026, it fell as soon as the stack came up. Switching to
    zero_effort from an armed state therefore goes to REST, never to that pair.
    """
    controllers = robot.controller_switcher_client.get_controller_list()
    active = {c.name for c in controllers if c.state == "active"}

    if TRAJECTORY in active:
        if controller_name in TORQUE_CONTROLLERS:
            # REST -> ARMED in one step: the overlay and the passthrough take over at the same
            # control cycle the fixed setpoint and the zero overlay are released.
            strict_switch(
                robot,
                activate=[PASSTHROUGH, controller_name],
                deactivate=[TRAJECTORY, ZERO_EFFORT],
            )
            return True
        if controller_name == ZERO_EFFORT:
            return True  # already at rest
        raise RuntimeError(
            f"refusing to switch to {controller_name} from rest: only a torque overlay "
            f"({TORQUE_CONTROLLERS}) or {ZERO_EFFORT} is meaningful here"
        )

    if PASSTHROUGH not in active:
        raise RuntimeError(
            "neither fri_position_passthrough_controller nor joint_trajectory_controller is "
            "active, so nothing owns the FRI position command. Refusing to switch. "
            f"Active controllers were: {sorted(active)}"
        )

    if controller_name == ZERO_EFFORT:
        # ARMED -> REST: the trajectory controller activates holding the measured position.
        overlays = sorted(set(TORQUE_CONTROLLERS) & active)
        strict_switch(
            robot,
            activate=[TRAJECTORY, ZERO_EFFORT],
            deactivate=[PASSTHROUGH, *overlays],
        )
        return True

    return robot.controller_switcher_client.switch_controller(
        controller_name,
        controllers_that_should_be_active=HOLD_ACTIVE,
    )


def active_controllers(robot: Robot) -> set[str]:
    """Names of the controllers the controller manager reports as active."""
    return {
        c.name
        for c in robot.controller_switcher_client.get_controller_list()
        if c.state == "active"
    }


def strict_switch(
    robot: Robot,
    *,
    activate: list[str],
    deactivate: list[str],
    timeout: float = 10.0,
) -> None:
    """Switch controllers with STRICT strictness, or raise.

    crisp_py's own switcher sends BEST_EFFORT, which reports success when only
    part of the switch happened. Swapping the position command interfaces
    between the passthrough and the trajectory controller must be all-or-nothing:
    a half-applied swap leaves the FRI position command unowned.
    """
    request = SwitchController.Request()
    request.activate_controllers = activate
    request.deactivate_controllers = deactivate
    request.strictness = SwitchController.Request.STRICT
    request.activate_asap = True
    request.timeout = rclpy.duration.Duration(seconds=int(timeout)).to_msg()

    client = robot.controller_switcher_client.switch_client
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError("controller_manager/switch_controller is not available.")

    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            raise RuntimeError("Timed out waiting for the controller switch to return.")
        time.sleep(0.01)

    if not future.result().ok:
        raise RuntimeError(
            f"STRICT switch failed. activate={activate} deactivate={deactivate}"
        )


def check_within_limits(degrees: list[float]) -> None:
    """Raise if any target angle is outside the iiwa14 R820 axis limits."""
    for index, (value, limit) in enumerate(zip(degrees, JOINT_LIMITS_DEG), start=1):
        if abs(value) > limit:
            raise ValueError(
                f"A{index} target {value:.1f} deg exceeds the +/-{limit:.0f} deg limit"
            )


def move_to_home(
    robot: Robot,
    degrees: list[float] | None = None,
    speed_deg_s: float = HOME_SPEED_DEG_S,
) -> None:
    """Move the arm to a joint configuration through the trajectory controller.

    This is the equivalent of crisp_py's Robot.home() for this cell. It cannot be
    Robot.home() itself: that calls the stock switcher, which would deactivate
    zero_effort_controller and leave the FRI torque overlay unowned while the arm
    is moving.

    The move commands positions. FRI hands them to Sunrise's joint impedance
    controller, at the 200 Nm/rad stiffness LBRServer sets, with the zero torque
    overlay still supplied by zero_effort_controller throughout.

    Run it in T1 with the enabling switch held and the workspace clear. Releasing
    the switch mid-move drops the FRI session and costs a full relaunch.
    """
    target = list(HOME_DEGREES if degrees is None else degrees)
    if len(target) != len(JOINT_LIMITS_DEG):
        raise ValueError(
            f"Expected {len(JOINT_LIMITS_DEG)} joint angles, got {len(target)}"
        )
    check_within_limits(target)

    active = active_controllers(robot)
    if PASSTHROUGH not in active and TRAJECTORY not in active:
        raise RuntimeError(
            f"neither {PASSTHROUGH} nor {TRAJECTORY} is active, so the FRI session is not in a "
            f"state to be handed the position command. Active controllers were: {sorted(active)}"
        )
    if ZERO_EFFORT not in active:
        raise RuntimeError(
            f"{ZERO_EFFORT} is not active. The torque overlay must be owned and zero for "
            f"the whole move. Active controllers were: {sorted(active)}"
        )
    overlapping = sorted(set(TORQUE_CONTROLLERS) & active)
    if overlapping:
        raise RuntimeError(
            f"Refusing to move: torque overlay controllers are active: {overlapping}. "
            f"Switch back to {ZERO_EFFORT} first."
        )

    start = np.rad2deg(np.asarray(robot.joint_values, dtype=float))
    delta = np.max(np.abs(np.asarray(target) - start))
    duration = max(delta / speed_deg_s, 2.0)

    print(
        f"largest joint delta {delta:.1f} deg, moving over {duration:.1f} s at {speed_deg_s} deg/s"
    )

    if TRAJECTORY not in active:
        strict_switch(robot, activate=[TRAJECTORY], deactivate=[PASSTHROUGH])
    # The trajectory controller is NOT handed back to the passthrough afterwards, whatever
    # happens. It used to be, and that left passthrough + zero_effort: a setpoint that follows
    # the measured position, so nothing holds the arm (see safe_switch). Staying here is REST:
    # the arm holds at its last commanded setpoint -- home, or wherever an aborted goal stopped
    # -- until safe_switch arms an overlay in one step.
    robot.joint_trajectory_controller_client.send_joint_config(
        robot.config.joint_names,
        list(np.deg2rad(target)),
        duration,
        blocking=True,
    )
