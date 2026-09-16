"""Carry Haply teleop poses onto /lbr/target_pose, with the safety envelope the cell lacks.

Two processes, because the two environments do not mix: the Haply plugin is ROS-free by
design (its pixi env has no rclpy) and this workspace's jazzy env has no lerobot. The clutch,
the anchoring and the frame mapping stay in the plugin where they are tested; this side
receives the resulting absolute pose over a UDP socket on loopback and republishes it as the
PoseStamped that `crisp_controllers/CartesianController` consumes.

It exists to be the thing that says no. The controller has NO target timeout, NO staleness
check and NO watchdog anywhere (verified by reading it): the last target latches forever, so
a teleop that dies mid-session leaves the arm holding a reference nobody is steering and
every published number stays healthy. Every refusal below is therefore enforced here, at the
last point before the wire:

* a first target after engage that is not already at the measured pose is REFUSED, not
  clamped. At 500 N/m a 10 cm error is 50 N arriving in one tick;
* per-tick translation and rotation are slew-clamped, so a dropped packet cannot become a
  lunge;
* the commanded position is held inside a workspace box in the base frame;
* a packet gap longer than --watchdog-ms stops publication, which under impedance means the
  arm holds its last commanded pose rather than going limp;
* SIGINT stops publication the same way, and says so.

Nothing here activates a controller. Bring the arm up, activate
`cartesian_impedance_controller` deliberately, and only then start this.

    pixi run -e jazzy python scripts/haply_teleop_bridge.py --dry-run   # publishes nothing
    pixi run -e jazzy python scripts/haply_teleop_bridge.py
"""

from __future__ import annotations

import argparse
import json
import signal
import socket
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

TARGET_TOPIC = "/lbr/target_pose"
CURRENT_POSE_TOPIC = "/lbr/current_pose"
JOINT_STATES_TOPIC = "/lbr/joint_states"

# The arm holds its pose under impedance, so "stop publishing" is a hold, not a release.
# Everything that goes wrong here resolves to that.
HOLD = "hold"


class Bridge(Node):
    """Republishes teleop poses, refusing the ones that would hurt."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Wire up the sockets and topics; publish nothing until a packet arrives."""
        super().__init__("haply_teleop_bridge")
        self.args = args
        self.current_pose: tuple[np.ndarray, Rotation] | None = None
        self.joint_positions: list[float] | None = None
        self.commanded: tuple[np.ndarray, Rotation] | None = None
        self.engaged = False
        self.last_packet_s: float | None = None
        self.refusals = 0
        self.published = 0
        self.clamped_ticks = 0

        self.pub = self.create_publisher(PoseStamped, TARGET_TOPIC, 1)
        self.create_subscription(
            PoseStamped, CURRENT_POSE_TOPIC, self._on_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joints, qos_profile_sensor_data
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((args.bind, args.port))
        self.sock.setblocking(False)
        self.reply_to: tuple[str, int] | None = None

        self.create_timer(1.0 / args.rate, self._tick)
        self.create_timer(1.0, self._report)
        self.get_logger().info(
            f"listening on udp://{args.bind}:{args.port}, publishing {TARGET_TOPIC} at "
            f"{args.rate:.0f} Hz"
            + (" — DRY RUN, nothing is published" if args.dry_run else "")
        )

    # -- inputs ---------------------------------------------------------------------

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        o = msg.pose.orientation
        self.current_pose = (
            np.array([p.x, p.y, p.z]),
            Rotation.from_quat([o.x, o.y, o.z, o.w]),
        )

    def _on_joints(self, msg: JointState) -> None:
        self.joint_positions = list(msg.position)

    def _drain(self) -> dict | None:
        """Take the NEWEST packet in the socket buffer and discard the backlog.

        A queued packet is a stale command: acting on the oldest first would walk the arm
        through a trail the operator's hand has already left.
        """
        latest = None
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break
            self.reply_to = addr
            try:
                latest = json.loads(data)
            except json.JSONDecodeError:
                continue
        return latest

    # -- the tick -------------------------------------------------------------------

    def _tick(self) -> None:
        packet = self._drain()
        now = time.monotonic()
        if packet is not None:
            self.last_packet_s = now
            self._consume(packet)
            self._reply()

        if self.last_packet_s is not None and (now - self.last_packet_s) * 1e3 > self.args.watchdog_ms:
            if self.engaged:
                self.get_logger().error(
                    f"no teleop packet for {self.args.watchdog_ms:.0f} ms — {HOLD}ing at the "
                    f"last commanded pose. The controller itself has no timeout, so this is "
                    f"the only thing that stops a dead teleop from latching a target."
                )
                self.engaged = False
            return

        if self.engaged and self.commanded is not None and not self.args.dry_run:
            self._publish(*self.commanded)

    def _consume(self, packet: dict) -> None:
        """Validate one teleop packet and update the commanded pose."""
        engaged = bool(packet.get("engaged", False))
        if not engaged:
            if self.engaged:
                self.get_logger().info(f"clutch released — {HOLD}ing at the last target")
            self.engaged = False
            return

        if self.current_pose is None:
            self.get_logger().warn_once(
                f"no {CURRENT_POSE_TOPIC} yet; refusing to command an arm whose pose is unknown"
            )
            return

        try:
            position = np.asarray(packet["position"], dtype=float)
            quat = np.asarray(packet["quaternion"], dtype=float)
        except (KeyError, TypeError, ValueError):
            self.get_logger().error(f"malformed packet, ignoring: {packet!r:.120}")
            return
        if position.shape != (3,) or quat.shape != (4,) or not np.all(np.isfinite(position)):
            self.get_logger().error("packet pose is not finite; ignoring")
            return
        rotation = Rotation.from_quat(quat / np.linalg.norm(quat))

        if not self.engaged:
            # The engage edge: the plugin anchors on the pose we last reported, so the first
            # commanded pose must already BE the measured one. Anything else is a jump, and a
            # jump at 500 N/m is a shove — so this refuses rather than clamping it smooth.
            gap = float(np.linalg.norm(position - self.current_pose[0]))
            ang = float((rotation * self.current_pose[1].inv()).magnitude())
            if gap > self.args.anchor_tol_m or ang > np.radians(self.args.anchor_tol_deg):
                self.refusals += 1
                self.get_logger().error(
                    f"REFUSED engage: first target is {gap * 1e3:.0f} mm / "
                    f"{np.degrees(ang):.1f} deg from the measured pose (limits "
                    f"{self.args.anchor_tol_m * 1e3:.0f} mm / {self.args.anchor_tol_deg:.0f} deg). "
                    f"The teleop anchored on a stale robot pose — restart the clutch."
                )
                return
            self.engaged = True
            self.commanded = (self.current_pose[0].copy(), self.current_pose[1])
            self.get_logger().info(
                f"clutch engaged at {np.round(self.commanded[0], 4).tolist()} "
                f"({gap * 1e3:.1f} mm / {np.degrees(ang):.2f} deg from measured)"
            )
            return

        self.commanded = self._limit(position, rotation)

    def _limit(self, position: np.ndarray, rotation: Rotation) -> tuple[np.ndarray, Rotation]:
        """Slew-clamp toward the request and hold it inside the workspace box."""
        assert self.commanded is not None
        prev_p, prev_r = self.commanded

        step = position - prev_p
        dist = float(np.linalg.norm(step))
        max_step = self.args.max_speed_m_s / self.args.rate
        if dist > max_step:
            position = prev_p + step * (max_step / dist)
            self.clamped_ticks += 1

        delta = rotation * prev_r.inv()
        ang = float(delta.magnitude())
        max_ang = np.radians(self.args.max_rot_deg_s) / self.args.rate
        if ang > max_ang:
            rotation = Rotation.from_rotvec(delta.as_rotvec() * (max_ang / ang)) * prev_r
            self.clamped_ticks += 1

        lo = np.array(self.args.workspace_min, dtype=float)
        hi = np.array(self.args.workspace_max, dtype=float)
        clipped = np.clip(position, lo, hi)
        if not np.allclose(clipped, position):
            self.clamped_ticks += 1
        return clipped, rotation

    # -- outputs --------------------------------------------------------------------

    def _publish(self, position: np.ndarray, rotation: Rotation) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        x, y, z, w = rotation.as_quat()
        msg.pose.orientation.x = float(x)
        msg.pose.orientation.y = float(y)
        msg.pose.orientation.z = float(z)
        msg.pose.orientation.w = float(w)
        self.pub.publish(msg)
        self.published += 1

    def _reply(self) -> None:
        """Send the arm's own state back, so the preview can draw the real robot."""
        if self.reply_to is None or self.current_pose is None:
            return
        payload = {
            "position": self.current_pose[0].tolist(),
            "quaternion": self.current_pose[1].as_quat().tolist(),
            "joints": self.joint_positions,
            "engaged": self.engaged,
            "published": self.published,
            "refusals": self.refusals,
            "clamped": self.clamped_ticks,
            "dry_run": bool(self.args.dry_run),
        }
        try:
            self.sock.sendto(json.dumps(payload).encode(), self.reply_to)
        except OSError:
            pass

    def _report(self) -> None:
        if not self.engaged:
            return
        assert self.commanded is not None and self.current_pose is not None
        err = float(np.linalg.norm(self.commanded[0] - self.current_pose[0]))
        self.get_logger().info(
            f"engaged · published {self.published} · tracking error {err * 1e3:.1f} mm · "
            f"clamped ticks {self.clamped_ticks} · refusals {self.refusals}"
        )


def main(argv: list[str] | None = None) -> int:
    """Run the bridge until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9301)
    parser.add_argument("--rate", type=float, default=200.0, help="publish rate, Hz")
    parser.add_argument("--base-frame", default="lbr_link_0")
    parser.add_argument("--dry-run", action="store_true", help="validate and log, publish nothing")
    parser.add_argument("--watchdog-ms", type=float, default=100.0)
    parser.add_argument("--anchor-tol-m", type=float, default=0.005)
    parser.add_argument("--anchor-tol-deg", type=float, default=3.0)
    # Deliberately slow. These are first-motion values for a cabinet on which no CRISP
    # torque controller has ever been active; raise them once the arm has been watched.
    parser.add_argument("--max-speed-m-s", type=float, default=0.05)
    parser.add_argument("--max-rot-deg-s", type=float, default=20.0)
    # A box around the home pose (ee at 0.596, 0.0, 0.494 in lbr_link_0), not the reachable
    # workspace: the point is to bound a first session, not to express the arm's limits.
    parser.add_argument("--workspace-min", type=float, nargs=3, default=[0.35, -0.35, 0.25])
    parser.add_argument("--workspace-max", type=float, nargs=3, default=[0.80, 0.35, 0.75])
    args = parser.parse_args(argv)

    rclpy.init()
    node = Bridge(args)

    def _stop(_sig: int, _frame: object) -> None:
        node.get_logger().info(f"interrupted — stopping publication, arm {HOLD}s last target")
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
