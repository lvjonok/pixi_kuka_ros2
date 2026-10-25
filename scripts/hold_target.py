"""Hold the pose the arm stands at as crisp's target, until ^C: a hand-push test for position_source.

With nothing on /lbr/target_pose crisp holds its activation pose by itself, but the passthrough's
`ik_target` goes stale after target_timeout_s and writes the measured position, so a push feels
the same in every mode. This publishes ONE pose -- /lbr/current_pose when it starts -- at 100 Hz,
so crisp and (under `ik_target`) Sunrise's spring both pull back to it. It never sends any other
pose, so starting it moves nothing beyond what crisp already holds.

It refuses unless cartesian_impedance_controller is active (ARMED) and nothing else publishes
on /lbr/target_pose. On ^C it stops publishing: crisp keeps the same pose, ik_target goes stale.

    pixi run -e jazzy python scripts/hold_target.py
"""

import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

NS = "/lbr"
TARGET_TOPIC = f"{NS}/target_pose"
CARTESIAN = "cartesian_impedance_controller"
RATE_HZ = 100.0


def wait(node: Node, done, timeout: float, what: str) -> None:
    deadline = time.monotonic() + timeout
    while not done():
        if time.monotonic() > deadline:
            raise SystemExit(f"no {what} within {timeout:.0f} s -- is the cell up?")
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> int:
    rclpy.init()
    node = Node("hold_target")
    try:
        client = node.create_client(ListControllers, f"{NS}/controller_manager/list_controllers")
        if not client.wait_for_service(timeout_sec=5.0):
            raise SystemExit("controller_manager is not up -- is the cell up?")
        future = client.call_async(ListControllers.Request())
        wait(node, future.done, 5.0, "controller list")
        active = {c.name for c in future.result().controller if c.state == "active"}
        if CARTESIAN not in active:
            raise SystemExit(f"{CARTESIAN} is not active: arm first (REST -> ARMED).")
        if node.count_publishers(TARGET_TOPIC) > 0:
            raise SystemExit(f"another node publishes on {TARGET_TOPIC}; close it first.")

        poses: list = []
        node.create_subscription(
            PoseStamped, f"{NS}/current_pose", poses.append, qos_profile_sensor_data
        )
        wait(node, lambda: bool(poses), 5.0, f"{NS}/current_pose")
        hold = poses[-1]
        p, o = hold.pose.position, hold.pose.orientation
        print(
            f"holding {TARGET_TOPIC} at p = [{p.x:.4f}, {p.y:.4f}, {p.z:.4f}] m, "
            f"q = [{o.x:.4f}, {o.y:.4f}, {o.z:.4f}, {o.w:.4f}] (frame '{hold.header.frame_id}'); "
            "^C to stop"
        )

        publisher = node.create_publisher(PoseStamped, TARGET_TOPIC, 1)
        message = PoseStamped()
        message.header.frame_id = hold.header.frame_id
        message.pose = hold.pose
        period = 1.0 / RATE_HZ
        next_time = time.monotonic()
        while rclpy.ok():
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            next_time += period
            time.sleep(max(0.0, next_time - time.monotonic()))
    except KeyboardInterrupt:
        print("\nstopped: crisp keeps the same pose; ik_target goes stale (measured).")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
