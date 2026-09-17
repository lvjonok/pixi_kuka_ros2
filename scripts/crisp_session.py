"""Inspect and drive the KUKA iiwa14 from crisp_py.

Read-only by default: prints controller states, joint positions and the
end-effector pose, and checks the gate that must hold before any torque overlay
is activated.

    pixi run -e jazzy session                       # status only
    pixi run -e jazzy session --home                # MOVES THE ARM to home
    pixi run -e jazzy session --switch joint_impedance_controller
    pixi run -e jazzy session --switch zero_effort_controller

--home is the only option here that moves the robot. It runs in T1 with the
enabling switch held, through kuka_crisp.move_to_home: a strict swap of the
position command interfaces to joint_trajectory_controller, a slow point-to-
point move with the zero torque overlay still owned, then a strict swap back.

Switching goes through kuka_crisp.safe_switch, which keeps
fri_position_passthrough_controller and estimated_wrench_interface active and
refuses to switch at all if the passthrough is not already active. Do not call
robot.controller_switcher_client.switch_controller directly: it drops every
active controller that is not named "*broadcaster".
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))

from kuka_crisp import (
    HOME_DEGREES,
    HOME_SPEED_DEG_S,
    ZERO_EFFORT,
    active_controllers,
    make_kuka_robot,
    move_to_home,
    safe_switch,
)

GATE = "fri_position_passthrough_controller"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--home",
        action="store_true",
        help="MOVES THE ARM to the home configuration through joint_trajectory_controller",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=HOME_SPEED_DEG_S,
        help=f"deg/s used to derive the --home trajectory duration (default {HOME_SPEED_DEG_S})",
    )
    parser.add_argument(
        "--switch",
        metavar="CONTROLLER",
        help="activate this controller via safe_switch (holds the passthrough active)",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    if args.home and args.switch:
        parser.error("--home and --switch are separate steps; run them one at a time.")

    print("connecting to /lbr ...")
    robot = make_kuka_robot()
    try:
        robot.wait_until_ready(timeout=args.timeout)
    except TypeError:
        robot.wait_until_ready()
    print("connected.\n")

    controllers = robot.controller_switcher_client.get_controller_list()
    active = {c.name for c in controllers if c.state == "active"}

    print("controllers:")
    for controller in sorted(controllers, key=lambda c: c.name):
        mark = (
            "  ACTIVE  " if controller.state == "active" else f"  {controller.state:<8}"
        )
        print(f"  {mark} {controller.name}")

    print(f"\nGATE {GATE}: {'PASS' if GATE in active else 'FAIL'}")

    q = np.asarray(robot.joint_values, dtype=float)
    print("\njoint positions [deg]:", np.array2string(np.rad2deg(q), precision=1))
    print(
        "home target     [deg]:", np.array2string(np.asarray(HOME_DEGREES), precision=1)
    )
    print(
        "delta from home [deg]:",
        np.array2string(np.rad2deg(q) - np.asarray(HOME_DEGREES), precision=1),
    )

    pose = robot.end_effector_pose
    print("\nEE position [m]:", np.array2string(np.asarray(pose.position), precision=3))

    if args.home:
        if GATE not in active:
            print(f"\nrefusing to move: {GATE} is not active.")
            robot.shutdown()
            return 1
        # Switch to zero-effort here, in the SAME process, immediately before moving. It is a
        # precondition of move_to_home -- the torque overlay must be owned and held at zero for
        # the whole move -- but running it as a separate command leaves the arm held only by
        # Sunrise's impedance about a mirrored measured position, which has no static stiffness.
        # The arm relaxes and sags in that gap. It did, on 17 Sep 2026, with a human waiting to
        # type the second command. The two are one operation and this is where they join.
        if ZERO_EFFORT not in active_controllers(robot):
            print(f"\nswitching to {ZERO_EFFORT} and moving immediately (no pause: the arm sags)")
            safe_switch(robot, ZERO_EFFORT)
        print("\nmoving to home. Keep the enabling switch held.")
        move_to_home(robot, speed_deg_s=args.speed)
        q = np.asarray(robot.joint_values, dtype=float)
        print("\njoint positions [deg]:", np.array2string(np.rad2deg(q), precision=1))
        print(
            "delta from home [deg]:",
            np.array2string(np.rad2deg(q) - np.asarray(HOME_DEGREES), precision=1),
        )
        print("active controllers now:")
        for controller in sorted(
            robot.controller_switcher_client.get_controller_list(), key=lambda c: c.name
        ):
            if controller.state == "active":
                print(f"  ACTIVE   {controller.name}")

    if args.switch:
        if GATE not in active:
            print(f"\nrefusing to switch: {GATE} is not active.")
            robot.shutdown()
            return 1
        print(f"\nswitching to {args.switch} ...")
        safe_switch(robot, args.switch)
        print("switched. re-reading controller states:")
        for controller in sorted(
            robot.controller_switcher_client.get_controller_list(), key=lambda c: c.name
        ):
            if controller.state == "active":
                print(f"  ACTIVE   {controller.name}")

    robot.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
