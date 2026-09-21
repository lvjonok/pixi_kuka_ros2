"""Read where the arm is standing and write it down as a home. Moves nothing.

    pixi run -e jazzy python scripts/capture_home.py --out <path.json> --chosen-by "..."

For a home that is chosen at the arm rather than computed -- the posture an experiment wants
every run to start from. Samples /lbr/joint_states for --seconds and REFUSES unless the arm
was still (worst per-joint spread under --max-spread-deg), because a home averaged over a
moving arm is a place the arm never was. Records the endpoint pose from /lbr/current_pose
beside the joints, with the frame the controller reports it in, so the file says where the
camera (or the flange) was and not only the joints.

The file is `session --home-file`'s input, and the same shape as lerobot_pickplace's
configs/home.deploy.json: `joint_names` and `position` in radians, plus provenance.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from kuka_crisp import JOINT_LIMITS_DEG
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

JOINT_NAMES = [f"lbr_A{i + 1}" for i in range(7)]


def main() -> int:
    """Sample, check stillness and limits, write."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--chosen-by", required=True, help="who picked this pose, and why")
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--max-spread-deg", type=float, default=0.05)
    parser.add_argument("--namespace", default="/lbr")
    parser.add_argument("--force", action="store_true", help="overwrite an existing file")
    args = parser.parse_args()

    if args.out.exists() and not args.force:
        print(f"refusing: {args.out} exists. A home is a decision; --force replaces it.")
        return 1

    rclpy.init()
    node = rclpy.create_node("capture_home")
    samples: list[list[float]] = []
    poses: list[PoseStamped] = []

    def on_joints(msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        if all(n in by_name for n in JOINT_NAMES):
            samples.append([by_name[n] for n in JOINT_NAMES])

    node.create_subscription(
        JointState, f"{args.namespace}/joint_states", on_joints, qos_profile_sensor_data
    )
    node.create_subscription(
        PoseStamped, f"{args.namespace}/current_pose", poses.append, qos_profile_sensor_data
    )

    deadline = time.monotonic() + 10.0
    while not samples and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not samples:
        print(f"refusing: no {args.namespace}/joint_states within 10 s -- is the stack up?")
        return 1
    samples.clear()
    poses.clear()
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    endpoint = None
    client = node.create_client(GetParameters, f"{args.namespace}/pose_broadcaster/get_parameters")
    if client.wait_for_service(timeout_sec=3.0):
        future = client.call_async(GetParameters.Request(names=["end_effector_frame"]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        if future.result() is not None and future.result().values:
            endpoint = future.result().values[0].string_value
    node.destroy_node()
    rclpy.shutdown()

    q = np.array(samples)
    spread_deg = np.degrees(q.max(axis=0) - q.min(axis=0))
    mean = q.mean(axis=0)
    degrees = np.degrees(mean)
    print(f"{len(q)} samples over {args.seconds:.1f} s")
    print("joints [deg]:", np.array2string(degrees, precision=2))
    print("spread [deg]:", np.array2string(spread_deg, precision=4))
    if spread_deg.max() > args.max_spread_deg:
        print(f"refusing: the arm moved (A{int(spread_deg.argmax()) + 1} spread "
              f"{spread_deg.max():.3f} deg > {args.max_spread_deg}). Let it settle.")
        return 1
    margin = np.array(JOINT_LIMITS_DEG) - np.abs(degrees)
    print("limit margin [deg]:", np.array2string(margin, precision=1))
    if margin.min() < 10.0:
        print(f"refusing: A{int(margin.argmin()) + 1} is {margin.min():.1f} deg from its limit; "
              "joint_limit_repulsion engages within 0.25 rad and would fight this home.")
        return 1

    record = {
        "note": "joint configuration to start policy runs from, captured at the arm",
        "provenance": {
            "captured": datetime.now(UTC).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "host": socket.gethostname(),
            "how": f"{len(q)} samples of {args.namespace}/joint_states over {args.seconds:.1f} s; "
                   f"worst per-joint spread {spread_deg.max():.4f} deg",
            "chosen_by": args.chosen_by,
            "limit_margin_deg": [round(float(m), 1) for m in margin],
        },
        "joint_names": JOINT_NAMES,
        "position": [round(float(v), 6) for v in mean],
        "degrees": [round(float(v), 3) for v in degrees],
    }
    if poses:
        p, o = poses[-1].pose.position, poses[-1].pose.orientation
        record["provenance"]["endpoint_at_this_pose"] = {
            "frame": f"{endpoint or 'unknown endpoint'} in {poses[-1].header.frame_id}",
            "xyz": [round(v, 6) for v in (p.x, p.y, p.z)],
            "quat_xyzw": [round(v, 6) for v in (o.x, o.y, o.z, o.w)],
        }
        print(f"endpoint {endpoint}: xyz {record['provenance']['endpoint_at_this_pose']['xyz']}")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(record, indent=2) + "\n")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
