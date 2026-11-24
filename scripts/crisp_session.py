"""Print the cell's controllers, state, joints and position; --home moves it, --switch switches.

    python scripts/crisp_session.py                       # prints only
    python scripts/crisp_session.py --home                # MOVES THE ARM to HOME_DEGREES
    python scripts/crisp_session.py --home --home-file F  # MOVES THE ARM to the joints in F
    python scripts/crisp_session.py --switch cartesian_impedance_controller   # REST -> ARMED
    python scripts/crisp_session.py --switch zero_effort_controller           # ARMED -> REST

--home runs in T1 with the enabling switch held, and leaves the arm at REST at the target. It
publishes /lbr/target_pose while it moves, so it is for REST: from ARMED it goes to REST first.
Without --home nothing is published on a target topic.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from kuka_crisp import HOME_SPEED_DEG_S, make_kuka_robot, move_to_home, safe_switch
from lbr_cell import (
    HOME_DEGREES,
    JOINTS,
    NS,
    ZERO_EFFORT,
    cell_state,
    controller_states,
    one_message,
    spinning,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def show_joints(node: Node, home_degrees: list[float]) -> None:
    message = one_message(node, JointState, f"{NS}/joint_states", qos_profile_sensor_data)
    position = dict(zip(message.name, message.position))
    degrees = np.rad2deg([position[joint] for joint in JOINTS])
    print("\njoint positions [deg]:", np.array2string(degrees, precision=1))
    print("home target     [deg]:", np.array2string(np.asarray(home_degrees), precision=1))
    print("delta from home [deg]:", np.array2string(degrees - home_degrees, precision=1))


def show_active(node: Node) -> None:
    print("active controllers now:")
    for name, state in sorted(controller_states(node).items()):
        if state == "active":
            print(f"  ACTIVE   {name}")


def run(node: Node, args: argparse.Namespace, home_degrees: list[float]) -> None:
    states = controller_states(node)
    print("controllers:")
    for name, state in sorted(states.items()):
        print(f"    {'ACTIVE  ' if state == 'active' else f'{state:<8}'} {name}")
    print(f"\ncell: {cell_state(states)}")
    show_joints(node, home_degrees)
    pose = one_message(node, PoseStamped, f"{NS}/current_pose", qos_profile_sensor_data).pose
    position = np.array([pose.position.x, pose.position.y, pose.position.z])
    print("\nEE position [m]:", np.array2string(position, precision=3))

    if args.home:
        if cell_state(states) != "REST":
            print(f"\nswitching to {ZERO_EFFORT} (REST)")
            safe_switch(node, ZERO_EFFORT)
        print("\nmoving to home. Keep the enabling switch held.")
        robot = make_kuka_robot()
        robot.wait_until_ready(timeout=15.0)
        move_to_home(robot, degrees=home_degrees, speed_deg_s=args.speed)
        show_joints(node, home_degrees)
        show_active(node)
    if args.switch:
        print(f"\nswitching to {args.switch} ...")
        safe_switch(node, args.switch)
        show_active(node)


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
        if captured["joint_names"] != list(JOINTS):
            parser.error(
                f"{args.home_file} names {captured['joint_names']}, not {', '.join(JOINTS)}"
            )
        home_degrees = [float(np.degrees(value)) for value in captured["position"]]
        print(f"home from {args.home_file}: {np.round(home_degrees, 2).tolist()} deg")

    rclpy.init()
    node = rclpy.create_node("crisp_session")
    try:
        with spinning(node):
            run(node, args, home_degrees)
    except (RuntimeError, ValueError, TimeoutError) as why:
        print(f"\n{why}")
        return 1
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
