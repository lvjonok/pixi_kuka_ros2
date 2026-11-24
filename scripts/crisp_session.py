"""The cell's controllers, joints and flange pose; with --home a joint move, with --switch a switch.

    python scripts/crisp_session.py                       # prints only
    python scripts/crisp_session.py --home                # MOVES THE ARM to HOME_DEGREES
    python scripts/crisp_session.py --home --home-file F  # MOVES THE ARM to the joints in F
    python scripts/crisp_session.py --switch cartesian_impedance_controller   # REST -> ARMED
    python scripts/crisp_session.py --switch zero_effort_controller           # ARMED -> REST

--home runs in T1 with the enabling switch held, and leaves the arm at REST at the target.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from crisp_py.robot import Robot
from kuka_crisp import (
    HOME_SPEED_DEG_S,
    active_controllers,
    make_kuka_robot,
    move_to_home,
    safe_switch,
)
from lbr_cell import HOME_DEGREES, JOINTS, ZERO_EFFORT, cell_state, controller_states


def show_joints(robot: Robot, home_degrees: list[float]) -> None:
    degrees = np.rad2deg(np.asarray(robot.joint_values, dtype=float))
    print("\njoint positions [deg]:", np.array2string(degrees, precision=1))
    print("home target     [deg]:", np.array2string(np.asarray(home_degrees), precision=1))
    print("delta from home [deg]:", np.array2string(degrees - home_degrees, precision=1))


def show_active(robot: Robot) -> None:
    print("active controllers now:")
    for name in sorted(active_controllers(robot)):
        print(f"  ACTIVE   {name}")


def run(robot: Robot, args: argparse.Namespace, home_degrees: list[float]) -> None:
    states = controller_states(robot.node)
    print("controllers:")
    for name, state in sorted(states.items()):
        print(f"    {'ACTIVE  ' if state == 'active' else f'{state:<8}'} {name}")
    print(f"\ncell: {cell_state(states)}")
    show_joints(robot, home_degrees)
    position = np.asarray(robot.end_effector_pose.position)
    print("\nEE position [m]:", np.array2string(position, precision=3))

    if args.home:
        if ZERO_EFFORT not in active_controllers(robot):
            print(f"\nswitching to {ZERO_EFFORT} (REST)")
            safe_switch(robot, ZERO_EFFORT)
        print("\nmoving to home. Keep the enabling switch held.")
        move_to_home(robot, degrees=home_degrees, speed_deg_s=args.speed)
        show_joints(robot, home_degrees)
        show_active(robot)
    if args.switch:
        print(f"\nswitching to {args.switch} ...")
        safe_switch(robot, args.switch)
        show_active(robot)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--home", action="store_true", help="MOVES THE ARM to the home joints")
    parser.add_argument(
        "--speed",
        type=float,
        default=HOME_SPEED_DEG_S,
        help=f"deg/s of the fastest joint in the --home move (default {HOME_SPEED_DEG_S})",
    )
    parser.add_argument(
        "--home-file",
        type=Path,
        help="JSON with joint_names and position (rad): the --home target in place of HOME_DEGREES",
    )
    parser.add_argument(
        "--switch",
        metavar="CONTROLLER",
        help="cartesian_impedance_controller, or zero_effort_controller",
    )
    args = parser.parse_args()
    if args.home and args.switch:
        parser.error("--home and --switch are separate steps; run them one at a time.")

    home_degrees = list(HOME_DEGREES)
    if args.home_file is not None:
        captured = json.loads(args.home_file.expanduser().read_text())
        names = [name.removesuffix(".pos") for name in captured["joint_names"]]
        if names != list(JOINTS):
            parser.error(f"{args.home_file} names {names}, not {', '.join(JOINTS)}")
        home_degrees = [float(np.degrees(value)) for value in captured["position"]]
        print(f"home from {args.home_file}: {np.round(home_degrees, 2).tolist()} deg")

    print("connecting to /lbr ...")
    robot = make_kuka_robot()
    try:
        robot.wait_until_ready(timeout=15.0)
        print("connected.\n")
        run(robot, args, home_degrees)
    except (RuntimeError, ValueError, TimeoutError) as why:
        print(f"\n{why}")
        return 1
    finally:
        robot.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
