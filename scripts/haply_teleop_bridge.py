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
import pathlib
import socket
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy._rclpy_pybind11 import RCLError
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))

TARGET_TOPIC = "/lbr/target_pose"
CURRENT_POSE_TOPIC = "/lbr/current_pose"
JOINT_STATES_TOPIC = "/lbr/joint_states"

# The nullspace target. CartesianController reads it as q_ref and drives the redundant degree of
# freedom toward it -- the elbow swivel, which the task term leaves entirely free.
#
# A packet MAY carry a `joints` field; when it does, it is republished here alongside the pose.
# Teleoperation never sends one, and that is exactly the gap this closes: over a 5.4-minute
# recorded session q_ref had a range of 0.0000 rad on all seven joints, so the elbow never moved
# and the visited configurations formed a 6-D sheet inside a 7-D space (participation ratio 3.49
# of 7). An autonomous excitation plan commands both halves, so the seventh dimension is excited
# on purpose rather than left to whatever posture the controller settled into.
#
# Same publisher-count trap as TARGET_TOPIC: CartesianController refuses every command on a topic
# carrying more than one publisher, and says so only in the controller_manager's log.
TARGET_JOINT_TOPIC = "/lbr/target_joint"

#: Joint names in the order CartesianController expects, matching config/controllers.yaml.
JOINT_NAMES = tuple(f"lbr_A{i + 1}" for i in range(7))

#: Hard joint limits for the iiwa 14 R820, radians. The commanded nullspace target is clamped
#: into these regardless of what arrives over the socket: q_ref is a spring anchor, and an anchor
#: outside the physical range pulls with a force that grows the harder the joint resists.
JOINT_LIMITS = np.radians(np.array([170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0]))

# The operator's state, republished onto ROS so a data recorder can see it.
#
# It already goes back to the teleop over UDP in _reply(), but that socket is a private channel
# between two processes on loopback. A recorder has no business reading it, and without this topic
# a recording cannot distinguish "the operator was driving" from "the arm was holding still under
# impedance" — two states whose rows look identical and mean entirely different things, because
# held rows are near-duplicates of each other and will dominate any stride-1 window set.
#
# It also carries the clamp counters, which is the part that matters for the science rather than
# the bookkeeping: --max-error-m clamps command error to bound the impedance force (500 N/m x
# 0.05 m = 25 N), and it fires during ordinary motion. The recorded command channel is therefore a
# SATURATED distribution, and a model fitted to it meets unclipped inputs the moment the envelope
# is widened. That has to be visible per row, not just in this process's log line.
#
# Float64MultiArray because it needs no new message package in either workspace. The cost is that
# the wire carries no field names, so a reordering here would silently move the engage flag into a
# clamp counter and every recorded value would still look plausible. STATUS_LABEL is the guard: it
# goes in layout.dim[0].label, the consumer compares it exactly and refuses to decode anything
# else. Change the field order -> bump the version in the label. The reader is
# iiwa_next.introspection.decode_teleop_status against iiwa_next.schema.TELEOP_STATUS_LABEL, and a
# test in that repo asserts the two strings match.
#
# Counters are cumulative rather than per-tick booleans: a burst of clamps inside one consumer tick
# would be invisible as a boolean, and a counter that only increases survives a dropped status
# message — the consumer sees a jump rather than a silent zero.
STATUS_TOPIC = "/haply_teleop/status"
STATUS_FIELDS = ("engaged", "engage_id", "clamped_lin", "clamped_rot", "clamped_box", "clamped_err")
STATUS_LABEL = "haply_teleop_status/1:" + ",".join(STATUS_FIELDS)

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
        self.packets = 0
        self._idle_ticks = 0
        # One counter per guard, not one shared counter. Lumping them together reports that
        # "something was clamped" and leaves you unable to tell a hand moving faster than the
        # speed limit from a wrist turning faster than the rotation limit from a target
        # pressed against the workspace wall — three different problems with three different
        # fixes, and the shared counter looks identical for all of them.
        self.clamped_lin = 0
        self.clamped_rot = 0
        self.clamped_box = 0
        self.clamped_err = 0
        self.peak_speed_m_s = 0.0
        self.peak_rot_deg_s = 0.0
        self.engage_id: int | None = None
        self._last_cmd_s: float | None = None
        self._engaged_since: float | None = None

        self.pub = self.create_publisher(PoseStamped, TARGET_TOPIC, 1)
        # Created unconditionally, published to only when a packet carries `joints`. Creating it
        # lazily would put topic discovery in the path of the first commanded sample, which is
        # the one sample that must not be late.
        self.joint_pub = self.create_publisher(JointState, TARGET_JOINT_TOPIC, 1)
        self.commanded_joints: np.ndarray | None = None
        self.clamped_joint = 0
        # Depth 1: a recorder wants the newest state, never a queued one it would then stamp with
        # a fresh arrival time. Unlike TARGET_TOPIC, a second publisher here is harmless — nothing
        # actuates on it — so no publisher count is checked.
        self.status_pub = self.create_publisher(Float64MultiArray, STATUS_TOPIC, 1)
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

        # CartesianController refuses EVERY command on a topic that has more than one
        # publisher, and says so only in the controller_manager's log — from here it looks
        # like a healthy bridge publishing into an arm that ignores it. Check once, loudly,
        # at the point where the cause is still obvious.
        time.sleep(0.5)  # let discovery settle before counting
        for topic in (TARGET_TOPIC, TARGET_JOINT_TOPIC):
            others = self.count_publishers(topic) - 1
            if others > 0:
                self.get_logger().error(
                    f"{others} OTHER publisher(s) on {topic}. The CRISP controller "
                    f"refuses all commands while a command topic has more than one publisher, "
                    f"so teleop will do nothing. Usual cause: a crisp_py process (a session "
                    f"script, or an orphaned one) still publishing its own target."
                )

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
        """One tick: decide what the arm is told, then say what state the operator was in.

        The decision is `_decide`, unchanged. The status publish is here, after it and outside it,
        so that it happens on EVERY path — including the watchdog's early return, which is exactly
        the moment a recorder most needs to see that engagement dropped.
        """
        self._decide()
        self._publish_status()

    def _decide(self) -> None:
        packet = self._drain()
        now = time.monotonic()
        if packet is not None:
            if self.packets == 0:
                self.get_logger().info("first teleop packet received — the link is up")
            self.packets += 1
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
        engage_id = packet.get("engage_id")
        if not engaged:
            if self.engaged:
                self.get_logger().info(f"clutch released — {HOLD}ing at the last target")
            self.engaged = False
            return

        # A new engage is whatever the teleop SAYS is a new engage, not what this side infers
        # from the boolean. Inferring it separately let a quick press-release-press be seen by
        # one end and missed by the other; the two then anchored on different poses and the
        # next packet arrived as a step of tens of millimetres in a single tick.
        if engage_id is not None and engage_id != self.engage_id:
            self.engage_id = engage_id
            self.engaged = False  # forces the anchor check below to run for this engage

        if self.current_pose is None:
            # `warn_once` is not an rclpy logger method; `once=True` is how rclpy spells it,
            # and getting it wrong would have raised AttributeError inside the timer callback
            # at exactly the moment the operator first engaged the clutch.
            self.get_logger().warning(
                f"no {CURRENT_POSE_TOPIC} yet; refusing to command an arm whose pose is unknown",
                once=True,
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
                # Throttled: the operator holds the clutch, so this fires every tick at
                # 200 Hz and an unthrottled version buries the heartbeat and the counter that
                # say what is actually going on. The count is in the heartbeat line.
                self.get_logger().error(
                    f"REFUSED engage: first target is {gap * 1e3:.0f} mm / "
                    f"{np.degrees(ang):.1f} deg from the measured pose (limits "
                    f"{self.args.anchor_tol_m * 1e3:.0f} mm / {self.args.anchor_tol_deg:.0f} deg). "
                    f"The teleop is anchored somewhere the arm is not — release the clutch, "
                    f"and check that the teleop is anchoring on the measured pose.",
                    throttle_duration_sec=2.0,
                )
                return
            self.engaged = True
            self.commanded = (self.current_pose[0].copy(), self.current_pose[1])
            # Reset the rate estimator too: the step across an engage is a re-anchor, not
            # motion, and letting it into the peak makes the measurement useless for choosing
            # a limit — which is what reported 22 m/s from a hand that never moved that fast.
            self._last_cmd_s = None
            self._engaged_since = time.monotonic()
            # Anchor the nullspace on the arm's MEASURED joints, for the same reason the pose is
            # anchored on the measured pose: q_ref is a spring anchor, and engaging with it set
            # somewhere the elbow is not applies a step torque at the first tick.
            self.commanded_joints = (
                np.asarray(self.joint_positions[:7], dtype=float)
                if self.joint_positions is not None and len(self.joint_positions) >= 7
                else None
            )
            self.get_logger().info(
                f"clutch engaged #{self.engage_id} at {np.round(self.commanded[0], 4).tolist()} "
                f"({gap * 1e3:.1f} mm / {np.degrees(ang):.2f} deg from measured)"
            )
            return

        self.commanded = self._limit(position, rotation)
        self.commanded_joints = self._limit_joints(packet.get("joints"))

    def _limit_joints(self, requested: object) -> np.ndarray | None:
        """Slew- and range-clamp a requested nullspace target.

        Args:
            requested: The packet's ``joints`` field, or ``None`` when it carried none.

        Returns:
            The clamped target, or the previous one when the packet carried nothing valid.

        ``q_ref`` is the anchor of a spring, not a position the arm is ordered to occupy. Two
        consequences, and both are the reason this is clamped rather than passed through:

        - a **step** in the anchor is a step in torque, so the anchor is rate-limited exactly as
          the pose is;
        - an anchor **outside** the joint's physical range pulls with a force that grows the
          harder the joint resists it, so it is held inside the limits whatever arrives.

        A malformed field is treated as "no update" rather than as a reason to stop: the nullspace
        holding its previous anchor is a safe, motionless state, and dropping the pose command
        because a seventh number was unreadable would be a worse failure than ignoring it.
        """
        if requested is None:
            return self.commanded_joints
        try:
            target = np.asarray(requested, dtype=float)
        except (TypeError, ValueError):
            self.get_logger().error("packet joints are not numeric; holding", throttle_duration_sec=2.0)
            return self.commanded_joints
        if target.shape != (7,) or not np.all(np.isfinite(target)):
            self.get_logger().error(
                "packet joints are not 7 finite numbers; holding", throttle_duration_sec=2.0
            )
            return self.commanded_joints

        clipped = np.clip(target, -JOINT_LIMITS + self.args.joint_margin_rad,
                          JOINT_LIMITS - self.args.joint_margin_rad)
        if not np.allclose(clipped, target):
            self.clamped_joint += 1

        previous = self.commanded_joints
        if previous is None:
            return clipped
        step = np.radians(self.args.max_joint_deg_s) / self.args.rate
        delta = np.clip(clipped - previous, -step, step)
        if not np.allclose(delta, clipped - previous):
            self.clamped_joint += 1
        return previous + delta

    def _limit(self, position: np.ndarray, rotation: Rotation) -> tuple[np.ndarray, Rotation]:
        """Slew-clamp toward the request and hold it inside the workspace box."""
        assert self.commanded is not None
        prev_p, prev_r = self.commanded

        step = position - prev_p
        dist = float(np.linalg.norm(step))
        max_step = self.args.max_speed_m_s / self.args.rate
        # The requested speed is recorded whether or not it was clamped: the limit that is
        # right is the one just above what the operator's hand actually does, and that number
        # cannot be read off a counter of how often the old guess was exceeded.
        #
        # Measured against elapsed time, not the nominal tick: the teleop sends at its own
        # loop rate, so dividing a whole packet's motion by this side's 5 ms tick reports a
        # speed several times what the hand did.
        now = time.monotonic()
        dt = None if self._last_cmd_s is None else now - self._last_cmd_s
        self._last_cmd_s = now
        # Ignore the first moments after an engage. The teleop re-anchors its own commanded
        # pose onto the measured one at that instant, so the commanded stream legitimately
        # steps by however far it had drifted — real for the clamp to absorb, but not motion,
        # and letting it into the peak is what reported 12 m/s from a hand doing 1.7.
        settled = self._engaged_since is not None and (now - self._engaged_since) > 0.1
        # Ignore intervals far shorter than the sender's own loop period. The teleop sends at
        # roughly 60 Hz, so a 2 ms gap means two packets arrived in a burst after a stall,
        # and dividing one packet's motion by that gap reports several m/s from a hand doing
        # two. Bursts are real and the clamps handle them; they just are not a hand speed.
        if dt is not None and dt > 0.008 and settled:
            self.peak_speed_m_s = max(self.peak_speed_m_s, dist / dt)
        if dist > max_step:
            position = prev_p + step * (max_step / dist)
            self.clamped_lin += 1

        delta = rotation * prev_r.inv()
        ang = float(delta.magnitude())
        max_ang = np.radians(self.args.max_rot_deg_s) / self.args.rate
        if dt is not None and dt > 0.008 and settled:
            self.peak_rot_deg_s = max(self.peak_rot_deg_s, np.degrees(ang) / dt)
        if ang > max_ang:
            rotation = Rotation.from_rotvec(delta.as_rotvec() * (max_ang / ang)) * prev_r
            self.clamped_rot += 1

        lo = np.array(self.args.workspace_min, dtype=float)
        hi = np.array(self.args.workspace_max, dtype=float)
        clipped = np.clip(position, lo, hi)
        if not np.allclose(clipped, position):
            self.clamped_box += 1

        # The guard that actually bounds force. The controller pulls with F = k*(x_d - x),
        # so what the arm can do to the world depends on the ERROR between command and
        # measurement, not on how fast the command moved: at 500 N/m a 100 mm gap is 50 N
        # however slowly it opened. Clamping the error caps the force by construction
        # (max_error_m * k), which lets the slew limits stay loose enough to teleoperate
        # with. It also makes the arm stop following rather than accumulate a pull if the
        # operator outruns it.
        if self.current_pose is not None:
            gap = clipped - self.current_pose[0]
            dist = float(np.linalg.norm(gap))
            if dist > self.args.max_error_m:
                clipped = self.current_pose[0] + gap * (self.args.max_error_m / dist)
                self.clamped_err += 1
            delta_r = rotation * self.current_pose[1].inv()
            ang_r = float(delta_r.magnitude())
            max_ang_r = np.radians(self.args.max_error_deg)
            if ang_r > max_ang_r:
                rotation = (
                    Rotation.from_rotvec(delta_r.as_rotvec() * (max_ang_r / ang_r))
                    * self.current_pose[1]
                )
                self.clamped_err += 1
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

        if self.commanded_joints is not None:
            joints = JointState()
            joints.header.stamp = msg.header.stamp
            joints.name = list(JOINT_NAMES)
            joints.position = [float(v) for v in self.commanded_joints]
            self.joint_pub.publish(joints)

    def _publish_status(self) -> None:
        """Publish the operator's state onto ROS, every tick, engaged or not.

        Every tick and unconditionally: a recorder needs "not engaged" just as much as "engaged",
        and publishing only while engaged would make a disengaged stretch indistinguishable from a
        bridge that had died — which is the difference between a recording with real hold data in it
        and a recording with no teleop state at all.

        Nothing here touches the command path. This publishes what the tick already decided; it
        cannot change what reaches the arm.
        """
        msg = Float64MultiArray()
        dim = MultiArrayDimension()
        # The field-order contract. A consumer that sees anything else must refuse to decode,
        # because a positional read of the wrong order produces plausible numbers in the wrong
        # columns and nothing downstream can tell.
        dim.label = STATUS_LABEL
        dim.size = len(STATUS_FIELDS)
        dim.stride = len(STATUS_FIELDS)
        msg.layout.dim = [dim]
        msg.data = [
            float(self.engaged),
            float(self.engage_id if self.engage_id is not None else -1),
            float(self.clamped_lin),
            float(self.clamped_rot),
            float(self.clamped_box),
            float(self.clamped_err),
        ]
        self.status_pub.publish(msg)

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
            "clamped": (
                self.clamped_lin + self.clamped_rot + self.clamped_box + self.clamped_err
            ),
            "clamped_err": self.clamped_err,
            "clamped_lin": self.clamped_lin,
            "clamped_rot": self.clamped_rot,
            "clamped_box": self.clamped_box,
            "peak_speed_m_s": round(self.peak_speed_m_s, 3),
            "peak_rot_deg_s": round(self.peak_rot_deg_s, 1),
            "dry_run": bool(self.args.dry_run),
        }
        try:
            self.sock.sendto(json.dumps(payload).encode(), self.reply_to)
        except OSError:
            pass

    def _report(self) -> None:
        if self.engaged:
            assert self.commanded is not None and self.current_pose is not None
            err = float(np.linalg.norm(self.commanded[0] - self.current_pose[0]))
            # In a dry run the arm does not move, so this gap is how far the operator has
            # carried the target from the anchor — not tracking lag. It only means lag once
            # the arm is actually following.
            gap = "gap to arm" if self.args.dry_run else "tracking error"
            self.get_logger().info(
                f"engaged · published {self.published} · {gap} {err * 1e3:.1f} mm · "
                f"clamped lin/rot/box/err "
                f"{self.clamped_lin}/{self.clamped_rot}/{self.clamped_box}/{self.clamped_err} · "
                f"peak {self.peak_speed_m_s:.2f} m/s, {self.peak_rot_deg_s:.0f} deg/s · "
                f"refusals {self.refusals}"
            )
            return

        # Idle heartbeat. Without it, "waiting for the operator to hold the clutch", "the
        # teleop is not running", and "this process is wedged" all look the same: a silent
        # terminal. Each of those needs a different fix, so each gets a different line.
        self._idle_ticks += 1
        if self._idle_ticks % self.args.heartbeat_s:
            return
        pose = "yes" if self.current_pose is not None else f"NO — is {CURRENT_POSE_TOPIC} up?"
        if self.packets == 0:
            self.get_logger().info(
                f"waiting · teleop packets: none yet on udp://{self.args.bind}:{self.args.port} "
                f"· robot pose: {pose}"
            )
        else:
            since = "" if self.last_packet_s is None else (
                f" ({time.monotonic() - self.last_packet_s:.1f} s ago)"
            )
            self.get_logger().info(
                f"waiting · {self.packets} packets received{since}, clutch released · "
                f"robot pose: {pose} · refusals {self.refusals}"
            )


def prepare_arm(controller: str, speed_deg_s: float, assume_yes: bool) -> None:
    """Home the arm and hand control to a CRISP controller, in the order this cell requires.

    The order is not cosmetic. `move_to_home` drives the POSITION interfaces through
    `joint_trajectory_controller`, and a CRISP torque overlay left active during that move
    would be fighting it — so any torque controller is put back to `zero_effort_controller`
    first, the arm is homed, and only then is the teleop controller raised.

    Uses this workspace's own `safe_switch` and `move_to_home` rather than raw controller
    calls: the former keeps the FRI passthrough and wrench interface active and refuses to
    switch without the passthrough, and the latter keeps the zero overlay owned throughout
    and swaps back even if the goal aborts. crisp_py's Robot.home() must not be used here —
    it deactivates the zero-effort controller and sends BEST_EFFORT.

    Raises:
        SystemExit: if the operator declines, or the arm is not ready.
    """
    from kuka_crisp import (
        HOME_DEGREES,
        TORQUE_CONTROLLERS,
        active_controllers,
        make_kuka_robot,
        move_to_home,
        safe_switch,
    )

    print(f"\n  This MOVES THE ARM: home to {HOME_DEGREES} deg at {speed_deg_s:.0f} deg/s,")
    print(f"  then hands control to {controller}.")
    print("  Workspace clear? Hand near the E-stop?")
    if not assume_yes:
        try:
            if input("  Press Enter to continue, Ctrl-C to abort: ").strip().lower() in {"n", "no"}:
                raise SystemExit("aborted")
        except (EOFError, KeyboardInterrupt):
            raise SystemExit("\naborted") from None

    robot = make_kuka_robot()
    robot.wait_until_ready(timeout=15.0)

    active = active_controllers(robot)
    for torque_controller in TORQUE_CONTROLLERS:
        if torque_controller in active:
            print(f"  {torque_controller} is active — returning to zero_effort_controller "
                  f"before a position move")
            safe_switch(robot, "zero_effort_controller")
            break

    print("  homing ...")
    move_to_home(robot, speed_deg_s=speed_deg_s)

    # Move crisp_py's OWN target onto the arm before raising the controller.
    #
    # The Robot object latches `_target_pose` to the current pose the first time one arrives
    # — which is at construction, BEFORE the homing move — and a 50 Hz timer republishes it
    # to target_pose for as long as the object lives (robot.py:108, :165-170, :357-358). So
    # it broadcasts the PRE-HOME pose continuously. Publishing the post-home pose from a
    # separate node does nothing: crisp_py overwrites it within 20 ms, the controller reads
    # that stale target on activation, and the arm drives back to wherever teleop last left
    # it at full stiffness. Measured on this bench: 66 mm back, after a correct home.
    #
    # Setting the target here means the stream the controller is already receiving carries
    # the pose the arm is at, so activation is a no-op whichever message it consumes.
    # BOTH targets, not just the pose. crisp_py publishes a joint target on its own 50 Hz
    # timer as well (robot.py:178-181), latched the same way, and under the Cartesian
    # controller that feeds q_ref — the nullspace reference. Parking only the pose leaves the
    # nullspace pulling the arm back toward the configuration it held before homing, which
    # reads as "it went back to the old pose" even though the Cartesian target is correct.
    pose = robot.end_effector_pose
    joints = robot.joint_values
    print(f"  parking crisp_py's targets at the measured pose "
          f"{np.round(pose.position, 4).tolist()} and joints "
          f"{np.round(np.degrees(joints), 1).tolist()} deg ...")
    robot.set_target(pose=pose)
    robot.set_target_joint(joints)
    time.sleep(0.3)  # several of crisp_py's own 50 Hz publications

    print(f"  raising {controller} ...")
    safe_switch(robot, controller)
    time.sleep(0.3)

    print(f"  ready — {controller} holds the arm where it stands; the clutch takes it from "
          f"there\n")
    # Tear the crisp_py robot down before the bridge starts: left alive, its executor thread
    # keeps spinning underneath. This takes rclpy down with it, which main() re-inits.
    robot.shutdown()


def main(argv: list[str] | None = None) -> int:
    """Run the bridge until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9301)
    parser.add_argument("--rate", type=float, default=200.0, help="publish rate, Hz")
    parser.add_argument("--base-frame", default="lbr_link_0")
    parser.add_argument("--dry-run", action="store_true", help="validate and log, publish nothing")
    parser.add_argument("--watchdog-ms", type=float, default=100.0)
    parser.add_argument("--heartbeat-s", type=int, default=5, help="idle status line period")
    # Off by default. A script that moves the arm the moment it launches is the kind of thing
    # that surprises somebody standing next to it, so moving is something you ask for.
    parser.add_argument(
        "--home",
        action="store_true",
        help="before bridging: home the arm and raise the teleop controller (MOVES THE ARM)",
    )
    parser.add_argument("--home-speed-deg-s", type=float, default=10.0)
    parser.add_argument(
        "--controller",
        default="cartesian_impedance_controller",
        help="the controller --home hands the arm to",
    )
    parser.add_argument("--yes", action="store_true", help="skip the --home confirmation")
    # Internal: --home re-execs this script with this flag to do the preparation in a child
    # process. See the comment at the call site for why it cannot share a process.
    parser.add_argument("--prepare-only", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--anchor-tol-m", type=float, default=0.005)
    parser.add_argument("--anchor-tol-deg", type=float, default=3.0)
    # Spike guards, not the force bound. Loose enough not to fight a hand (measured peak
    # around 2 m/s on this bench), tight enough that one bad packet cannot become a lunge.
    parser.add_argument("--max-speed-m-s", type=float, default=1.0)
    parser.add_argument("--max-rot-deg-s", type=float, default=360.0)
    # The nullspace anchor's slew limit. Well under the arm's rated joint speeds (75-135 deg/s)
    # because this moves the elbow through a 3 Nm spring rather than commanding a position: the
    # anchor outrunning the joint just winds the spring up to its torque clamp and stays there.
    parser.add_argument("--max-joint-deg-s", type=float, default=45.0)
    # Keep the anchor off the hard stops. The controller's own joint_limit_repulsion engages
    # within 0.15 rad and would spend the session fighting an anchor parked inside that band.
    parser.add_argument("--joint-margin-rad", type=float, default=0.15)
    # The force bound, and the one to think about: F_max = max_error_m * task stiffness.
    # At the configured 500 N/m, 0.05 m is 25 N; 20 Nm/rad and 15 deg is 5.2 Nm.
    parser.add_argument("--max-error-m", type=float, default=0.05)
    parser.add_argument("--max-error-deg", type=float, default=15.0)
    # A box around the home pose (ee at 0.596, 0.0, 0.494 in lbr_link_0), not the reachable
    # workspace: the point is to bound a first session, not to express the arm's limits.
    parser.add_argument("--workspace-min", type=float, nargs=3, default=[0.35, -0.35, 0.25])
    parser.add_argument("--workspace-max", type=float, nargs=3, default=[0.80, 0.35, 0.75])
    args = parser.parse_args(argv)

    if args.prepare_only:
        rclpy.init()
        prepare_arm(args.controller, args.home_speed_deg_s, args.yes)
        return 0

    if args.home:
        if args.dry_run:
            # Refusing rather than quietly skipping: --dry-run means "command nothing", and
            # homing is a command. Silently honouring one half of the pair would move an arm
            # somebody believed was safe to leave alone.
            raise SystemExit("--home moves the arm and --dry-run says not to; pick one")
        # In a CHILD PROCESS, not here. crisp_py's Robot publishes to target_pose, and its
        # shutdown races with its own executor thread, leaving that publisher in the graph.
        # CartesianController refuses every command while a topic has more than one
        # publisher ("SAFETY WARNING: Multiple command sources detected") — so an orphaned
        # crisp_py publisher makes this bridge the second one and silently disables teleop
        # entirely. A child process takes its DDS entities to the grave with it.
        child = subprocess.run(
            [sys.executable, str(pathlib.Path(__file__).resolve()), "--prepare-only",
             "--controller", args.controller,
             "--home-speed-deg-s", str(args.home_speed_deg_s)]
            + (["--yes"] if args.yes else []),
            check=False,
        )
        if child.returncode != 0:
            return child.returncode

    rclpy.init()
    node = Bridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RCLError:
        # A SIGTERM lands between the context being invalidated and spin noticing, and spin
        # then fails to build its wait set. Same clean stop, different exception.
        pass
    except ExternalShutdownException:
        # rclpy installs its own signal handling, so Ctrl-C shuts the context down out from
        # under spin and this is the normal exit path, not a fault. Left uncaught it prints a
        # traceback on every clean stop, which trains the operator to ignore tracebacks.
        pass
    finally:
        # Plain stderr, not the node logger: by this point the context can already be
        # invalid, and rosout then fails to publish and prints its own error over the top of
        # the one line the operator actually needs to read on the way out.
        print(
            f"stopping publication — the arm {HOLD}s its last commanded pose under impedance",
            file=sys.stderr,
        )
        node.destroy_node()
        # Already shut down when we got here via ExternalShutdownException; calling it again
        # raises RCLError from the C layer.
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
