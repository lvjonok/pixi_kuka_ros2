"""Bench check of the passthrough on the mock stack.

It drives the plugin as the controller manager runs it: arming and resting, the /lbr/target_pose
subscription, the IK against one computed here, both clamps, the stale fallback, two publishers
and ~/state.

    pixi run -e jazzy mock-loopback                      # terminal 1
    pixi run -e jazzy check-mock                         # terminal 2

mock_components/GenericSystem copies each position command into the position state at the next
read, so q_meas(k) = q_out(k-1):

* the arm follows the passthrough's output, and FK of /lbr/joint_states is where the IK went;
* |q_out - q_meas| is one update's step: gap_clamped fires, but the gap cannot bind for longer
  than an update.

Refuses to run unless ROS_DOMAIN_ID is 77, the RMW is CycloneDDS on this repository's loopback
file, and the robot_description is mock_components/GenericSystem. Leaves the stack at REST. Exits
0 when every check passed.
"""

import itertools
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from lbr_cell import (
    ARMED,
    BASE_FRAME,
    HOME_DEGREES,
    JOINTS,
    NS,
    PASSTHROUGH,
    REST,
    TRAJECTORY,
    call,
    cell_state,
    controller_states,
    one_message,
    spinning,
    switch,
)
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectoryPoint

REPO = Path(__file__).resolve().parents[1]
LOOPBACK = REPO / "config" / "cyclonedds_loopback.xml"
N = len(JOINTS)
# ~/state[0] under position_source ik_target.
IK_TARGET = 2
PARAMETERS = (
    "end_effector_frame",
    "target_timeout_s",
    "ik_damping",
    "nullspace_gain",
    "joint_limit_margin_rad",
    "max_joint_step_rad",
    "sunrise_joint_stiffness",
    "max_spring_torque_nm",
)
TARGET_RATE_HZ = 100.0
REACH_MM = 3.0
REACH_DEG = 2.0
EPS = 1e-12


def environment_problem() -> str | None:
    """Why this process could reach beyond the loopback domain 77, or None."""
    if os.environ.get("ROS_DOMAIN_ID") != "77":
        return f"ROS_DOMAIN_ID is {os.environ.get('ROS_DOMAIN_ID')!r}, not '77'"
    # Any other RMW ignores CYCLONEDDS_URI and uses every interface.
    rmw = rclpy.get_rmw_implementation_identifier()
    if rmw != "rmw_cyclonedds_cpp":
        return f"the RMW is {rmw!r}, not rmw_cyclonedds_cpp (RMW_IMPLEMENTATION)"
    uri = os.environ.get("CYCLONEDDS_URI", "")
    if not uri.startswith("file://") or Path(uri.removeprefix("file://")).resolve() != LOOPBACK:
        return f"CYCLONEDDS_URI is {uri!r}, not file://{LOOPBACK}"
    return None


@dataclass
class Results:
    rows: list[tuple[str, bool, str]] = field(default_factory=list)

    def check(self, name: str, ok: bool, detail: str) -> bool:
        self.rows.append((name, bool(ok), detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)
        return bool(ok)

    def summary(self) -> int:
        failed = [r for r in self.rows if not r[1]]
        print(f"\n{len(self.rows) - len(failed)}/{len(self.rows)} checks passed")
        for name, _, detail in failed:
            print(f"  FAIL {name}: {detail}")
        return 1 if failed else 0


@dataclass
class State:
    t: float  # receive time, monotonic
    mode: int
    stale: bool
    q_meas: np.ndarray
    q_ik: np.ndarray
    q_out: np.ndarray
    gap: np.ndarray
    step: np.ndarray


class Mock(Node):
    def __init__(self) -> None:
        super().__init__("position_source_mock_check")
        self.lock = threading.Lock()
        self.states: list[State] = []
        self.joints: list[tuple[float, np.ndarray]] = []
        self.logs: list[tuple[float, str]] = []
        self.create_subscription(
            Float64MultiArray,
            f"{NS}/{PASSTHROUGH}/state",
            self._on_state,
            QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(JointState, f"{NS}/joint_states", self._on_joints, 100)
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        self.target_pub = self.create_publisher(PoseStamped, f"{NS}/target_pose", 1)
        self.trajectory = ActionClient(
            self, FollowJointTrajectory, f"{NS}/{TRAJECTORY}/follow_joint_trajectory"
        )

    def _on_state(self, message: Float64MultiArray) -> None:
        d = np.asarray(message.data, dtype=float)
        b = [d[2 + k * N : 2 + (k + 1) * N] for k in range(5)]
        state = State(time.monotonic(), int(d[0]), d[1] != 0, *b)
        with self.lock:
            self.states.append(state)

    def _on_joints(self, message: JointState) -> None:
        index = {n: i for i, n in enumerate(message.name)}
        q = np.array([message.position[index[j]] for j in JOINTS])
        with self.lock:
            self.joints.append((time.monotonic(), q))

    def _on_log(self, message: Log) -> None:
        if PASSTHROUGH in message.name:
            with self.lock:
                self.logs.append((time.monotonic(), message.msg))

    def mark(self) -> tuple[int, int, int]:
        with self.lock:
            return len(self.states), len(self.joints), len(self.logs)

    def since(self, mark: tuple[int, int, int]):
        with self.lock:
            return (
                list(self.states[mark[0] :]),
                list(self.joints[mark[1] :]),
                list(self.logs[mark[2] :]),
            )

    def q(self) -> np.ndarray:
        with self.lock:
            return self.joints[-1][1].copy()

    def cell(self) -> str:
        return cell_state(controller_states(self))

    def parameters(self) -> dict:
        answer = call(
            self,
            GetParameters,
            f"{NS}/{PASSTHROUGH}/get_parameters",
            GetParameters.Request(names=list(PARAMETERS)),
        )
        # rclcpp answers with no value at all when one of the names is not declared.
        if len(answer.values) != len(PARAMETERS):
            raise RuntimeError(f"{PASSTHROUGH} does not declare {', '.join(PARAMETERS)}")
        return dict(zip(PARAMETERS, map(parameter_value_to_python, answer.values)))

    def try_switch(self, *, activate: tuple[str, ...], deactivate: tuple[str, ...]) -> str:
        """One STRICT switch. Empty, or why the controller manager refused it."""
        try:
            switch(self, activate=activate, deactivate=deactivate)
        except RuntimeError as why:
            return str(why)
        return ""

    def _wait(self, future, timeout: float = 10.0):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError("the trajectory action did not answer in time")
            time.sleep(0.005)
        return future.result()

    def move_joints(self, q: np.ndarray, seconds: float) -> bool:
        if not self.trajectory.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("follow_joint_trajectory is not up")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        point.velocities = [0.0] * N
        point.time_from_start = rclpy.duration.Duration(seconds=seconds).to_msg()
        goal.trajectory.points = [point]
        handle = self._wait(self.trajectory.send_goal_async(goal))
        if not handle.accepted:
            return False
        result = self._wait(handle.get_result_async(), timeout=seconds + 10.0)
        return result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def publish_target(self, pose: pin.SE3) -> None:
        message = PoseStamped()
        message.header.frame_id = BASE_FRAME
        message.header.stamp = self.get_clock().now().to_msg()
        p = pose.translation
        x, y, z, w = Rotation.from_matrix(pose.rotation).as_quat()
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, p)
        message.pose.orientation.x = float(x)
        message.pose.orientation.y = float(y)
        message.pose.orientation.z = float(z)
        message.pose.orientation.w = float(w)
        self.target_pub.publish(message)

    def stream(self, pose: pin.SE3, seconds: float) -> float:
        """Publish ``pose`` at TARGET_RATE_HZ for ``seconds``; the monotonic time of the last."""
        period = 1.0 / TARGET_RATE_HZ
        end = time.monotonic() + seconds
        last = time.monotonic()
        while time.monotonic() < end:
            self.publish_target(pose)
            last = time.monotonic()
            time.sleep(period)
        return last


class Kinematics:
    """FK and the IK law on the stack's robot_description, computed here with pinocchio."""

    def __init__(self, urdf: str, frame: str) -> None:
        self.model = pin.buildModelFromXML(urdf)
        self.data = self.model.createData()
        self.frame = self.model.getFrameId(frame)
        if self.frame >= self.model.nframes:
            raise RuntimeError(f"no frame {frame} in the robot_description")
        self.idx = [self.model.joints[self.model.getJointId(j)].idx_q for j in JOINTS]
        self.lower = self.model.lowerPositionLimit[self.idx]
        self.upper = self.model.upperPositionLimit[self.idx]

    def _q(self, q: np.ndarray) -> np.ndarray:
        full = pin.neutral(self.model)
        full[self.idx] = q
        return full

    def fk(self, q: np.ndarray) -> pin.SE3:
        pin.framesForwardKinematics(self.model, self.data, self._q(q))
        return self.data.oMf[self.frame].copy()

    def _error_jacobian(self, q: np.ndarray, target: pin.SE3) -> tuple[np.ndarray, np.ndarray]:
        full = self._q(q)
        pin.framesForwardKinematics(self.model, self.data, full)
        current = self.data.oMf[self.frame]
        e = np.concatenate(
            [
                target.translation - current.translation,
                pin.log3(target.rotation @ current.rotation.T),
            ]
        )
        J = pin.computeFrameJacobian(
            self.model, self.data, full, self.frame, pin.LOCAL_WORLD_ALIGNED
        )[:, self.idx]
        return e, J

    def ik_step(
        self, q: np.ndarray, target: pin.SE3, posture: np.ndarray, values: dict
    ) -> np.ndarray:
        """One update of the IK law (docs/position_source.md) from q, target unfiltered."""
        e, J = self._error_jacobian(q, target)
        lam = values["ik_damping"]
        pinv = J.T @ np.linalg.inv(J @ J.T + lam * lam * np.eye(6))
        pull = values["nullspace_gain"] * (posture - q)
        dq = pinv @ e + (np.eye(N) - pinv @ J) @ pull
        margin = values["joint_limit_margin_rad"]
        return np.clip(q + dq, self.lower + margin, self.upper - margin)

    def ik(self, q: np.ndarray, target: pin.SE3, iterations: int = 500) -> np.ndarray | None:
        """Damped least squares from q: the joint solution of a reachable target, or None."""
        q = q.copy()
        for _ in range(iterations):
            e, J = self._error_jacobian(q, target)
            if np.linalg.norm(e) < 1e-9:
                return q if np.all(q > self.lower) and np.all(q < self.upper) else None
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
            q = np.clip(q + dq, self.lower, self.upper)
        return None


def pose_error(a: pin.SE3, b: pin.SE3) -> tuple[float, float]:
    """(mm, deg) between two poses."""
    mm = 1000.0 * float(np.linalg.norm(a.translation - b.translation))
    deg = math.degrees(float(np.linalg.norm(pin.log3(a.rotation @ b.rotation.T))))
    return mm, deg


def offset(pose: pin.SE3, dp=(0.0, 0.0, 0.0), rotvec_deg=(0.0, 0.0, 0.0)) -> pin.SE3:
    """``pose`` moved by dp (m) and rotated by rotvec (deg), both in base axes."""
    R = Rotation.from_rotvec(np.radians(rotvec_deg)).as_matrix()
    return pin.SE3(R @ pose.rotation, pose.translation + np.asarray(dp))


def consecutive(states: list[State]) -> list[tuple[State, State]]:
    """Pairs of ~/state messages from successive updates: on the mock q_meas(k) = q_out(k-1).

    The realtime publisher skips a message when it cannot take its lock, which breaks the
    identity: a per-update check uses only the pairs that hold it and in which the output moved.
    """
    return [
        (a, b)
        for a, b in itertools.pairwise(states)
        if np.all(np.abs(b.q_meas - a.q_out) <= 1e-12) and np.any(b.q_out != a.q_out)
    ]


def fmt(values) -> str:
    return "[" + ", ".join(f"{v:.4g}" for v in values) + "]"


def stationary(joints: list[tuple[float, np.ndarray]]) -> float:
    """Largest change of any joint over the recording, rad."""
    if len(joints) < 2:
        return float("nan")
    q = np.array([j[1] for j in joints])
    return float(np.max(np.abs(q - q[0])))


class Gate:
    def __init__(self, node: Mock, results: Results) -> None:
        self.node = node
        self.r = results

    def setup(self) -> bool:
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        try:
            urdf = one_message(self.node, String, f"{NS}/robot_description", latched).data
        except RuntimeError as why:
            print(f"REFUSED: {why}")
            return False
        if "mock_components/GenericSystem" not in urdf:
            print("REFUSED: the robot_description is not mock_components/GenericSystem")
            return False
        deadline = time.monotonic() + 10.0
        while not self.node.joints and time.monotonic() < deadline:
            time.sleep(0.05)
        if not self.node.joints:
            print("REFUSED: no /lbr/joint_states: is the mock up?")
            return False
        if self.node.cell() != "REST":
            print(f"REFUSED: not at REST ({self.node.cell()})")
            return False
        self.values = self.node.parameters()
        self.gap = np.array(self.values["max_spring_torque_nm"]) / np.array(
            self.values["sunrise_joint_stiffness"]
        )
        self.max_step = np.array(self.values["max_joint_step_rad"])
        self.kin = Kinematics(urdf, self.values["end_effector_frame"])
        print(f"gap {fmt(self.gap)} rad, step {fmt(self.max_step)} rad per update")
        ok = self.node.move_joints(np.radians(HOME_DEGREES), 4.0)
        time.sleep(0.3)
        self.r.check(
            "setup: home through joint_trajectory_controller at REST",
            ok and np.allclose(self.node.q(), np.radians(HOME_DEGREES), atol=1e-3),
            f"q = {fmt(np.degrees(self.node.q()))} deg",
        )
        return True

    def arm(self, label: str) -> bool:
        before = self.node.cell()
        refused = self.node.try_switch(activate=ARMED, deactivate=REST)
        after = self.node.cell()
        return self.r.check(
            f"{label}: REST -> ARMED (STRICT switch)",
            not refused and after.startswith("ARMED"),
            f"{before} -> {after}{'; ' + refused if refused else ''}",
        )

    def rest(self, label: str) -> bool:
        mark = self.node.mark()
        refused = self.node.try_switch(activate=REST, deactivate=ARMED)
        time.sleep(0.2)
        _, _, logs = self.node.since(mark)
        failed = [m for _, m in logs if "Deactivating without" in m or "Failed to leave" in m]
        return self.r.check(
            f"{label}: ARMED -> REST, no error from the passthrough's on_deactivate",
            not refused and self.node.cell() == "REST" and not failed,
            f"{self.node.cell()}; "
            + (f"on_deactivate: {failed[0]}" if failed else "no on_deactivate error")
            + (f"; {refused}" if refused else ""),
        )

    def logged(self, label: str, logs: list[tuple[float, str]], text: str) -> None:
        self.r.check(
            label,
            any(text in m for _, m in logs),
            next((m for _, m in logs if "position_source" in m), "no log line on /rosout"),
        )

    def _bounds(self, label: str, states: list[State]) -> None:
        gap = max((np.max(np.abs(s.q_out - s.q_meas) - self.gap) for s in states), default=-1)
        worst = np.max([np.abs(s.q_out - s.q_meas) for s in states], axis=0)
        pairs = consecutive(states)
        steps = (
            np.max([np.abs(b.q_out - a.q_out) for a, b in pairs], axis=0) if pairs else np.zeros(N)
        )
        self.r.check(
            f"{label}: |q_out - q_meas| <= gap in every message",
            gap <= EPS,
            f"max per joint {fmt(worst)} rad vs gap {fmt(self.gap)}",
        )
        self.r.check(
            f"{label}: |q_out(k) - q_out(k-1)| <= max_joint_step_rad",
            bool(np.all(steps <= self.max_step + EPS)),
            f"max per joint over {len(pairs)} successive updates {fmt(steps)} rad",
        )

    def _first_within(self, states: list[State], target: pin.SE3) -> int | None:
        """The first message whose q_meas is within 1 mm and 0.5 deg of the target."""
        for i, s in enumerate(states):
            mm, deg = pose_error(self.kin.fk(s.q_meas), target)
            if mm < 1.0 and deg < 0.5:
                return i
        return None

    def reach(self, label: str, target: pin.SE3, seconds: float) -> None:
        mark = self.node.mark()
        t0 = time.monotonic()
        self.node.stream(target, seconds)
        time.sleep(0.05)
        states, _, _ = self.node.since(mark)
        mm, deg = pose_error(self.kin.fk(self.node.q()), target)
        stale = sum(int(s.stale) for s in states if s.t >= t0 + 0.05)
        there = self._first_within(states, target)
        settled = float("nan") if there is None else states[there].t - t0
        self.r.check(
            f"{label}: FK(/lbr/joint_states) reaches the target",
            mm < REACH_MM and deg < REACH_DEG and stale == 0,
            f"{mm:.3f} mm / {deg:.3f} deg after {seconds:.1f} s (within 1 mm / 0.5 deg at "
            f"{settled * 1000:.0f} ms), {stale} stale messages while streaming",
        )
        self._bounds(label, states)

    def run(self) -> None:
        here0 = self.kin.fk(self.node.q())
        self.node.stream(offset(here0, (0.0, 0.0, 0.05)), 0.3)
        time.sleep(0.2)
        armed = self.node.mark()
        self.arm_ignores_old_target("first arm")

        p0 = self.kin.fk(self.node.q())
        targets = [
            ("+30 mm x", offset(p0, (0.03, 0.0, 0.0))),
            ("+30 mm x, -20 mm z, 10 deg about x", offset(p0, (0.03, 0.0, -0.02), (10, 0, 0))),
            ("-20 mm y, +15 mm z, 5 deg y, -8 deg z", offset(p0, (0.0, -0.02, 0.015), (0, 5, -8))),
            ("back to the start pose", p0),
        ]
        for name, pose in targets:
            self.reach(name, pose, 2.0)

        self.far(p0)
        self.stale_and_resume(p0)
        self.two_publishers(p0)
        states, _, _ = self.node.since(armed)
        modes = sorted({s.mode for s in states})
        self.r.check(
            "~/state: the first value is that of ik_target in every message",
            modes == [IK_TARGET],
            f"{len(states)} messages while ARMED, first value {modes}",
        )

        if not self.rest("first arm"):
            return
        here = self.kin.fk(self.node.q())
        self.node.stream(offset(here, (0.0, 0.05, 0.0)), 0.5)
        time.sleep(0.2)
        self.arm_ignores_old_target("re-arm")
        here = self.kin.fk(self.node.q())
        target = offset(here, (0.0, 0.0, 0.01))
        mark = self.node.mark()
        t0 = time.monotonic()
        self.reach("re-arm +10 mm z", target, 1.5)
        self.restart_check("re-arm", self.node.since(mark)[0], t0, target)
        self.rest("re-arm")

    def arm_ignores_old_target(self, label: str) -> None:
        """Arm after a target was published at REST: that target must never be used."""
        q_rest = self.node.q()
        self.posture = q_rest
        mark = self.node.mark()
        if not self.arm(label):
            return
        time.sleep(0.5)
        states, joints, logs = self.node.since(mark)
        first = states[0] if states else None
        self.logged(
            f"{label}: activation logs ik_target with its frames and topic",
            logs,
            "position_source: ik_target (lbr_link_ee in lbr_link_0 from '/lbr/target_pose'",
        )
        self.r.check(
            f"{label}: starts from the measured position",
            first is not None
            and np.array_equal(first.q_out, q_rest)
            and np.array_equal(first.q_ik, q_rest),
            "first ~/state q_out = q_ik = q at REST"
            if first is not None and np.array_equal(first.q_out, q_rest)
            else f"first q_out - q_rest {fmt(first.q_out - q_rest) if first else 'n/a'}",
        )
        moved = max(stationary(joints), float(np.max(np.abs(self.node.q() - q_rest))))
        self.r.check(
            f"{label}: a target published before activation is ignored",
            moved == 0.0 and all(s.stale and np.array_equal(s.q_out, s.q_meas) for s in states),
            f"{len(states)} messages over 0.5 s, all stale={int(all(s.stale for s in states))} "
            f"with q_out = q_meas; largest joint change {moved:.3g} rad",
        )

    def far(self, p0: pin.SE3) -> None:
        q = self.node.q()
        for dp in ((0.0, 0.3, 0.0), (0.0, -0.3, 0.0), (0.0, 0.0, -0.3), (-0.3, 0.0, 0.0)):
            target = offset(p0, dp)
            solution = self.kin.ik(q, target)
            if solution is not None:
                break
        else:
            self.r.check("far: a reachable 0.3 m target", False, "none found")
            return
        print(
            f"  far target {dp} m from the start pose; its IK is "
            f"{fmt(np.degrees(solution - q))} deg away"
        )
        mark = self.node.mark()
        t0 = time.monotonic()
        self.node.stream(target, 4.0)
        time.sleep(0.05)
        states, _, _ = self.node.since(mark)
        gap_count = np.sum([s.gap for s in states], axis=0)
        step_count = np.sum([s.step for s in states], axis=0)
        beyond = np.max([np.abs(s.q_ik - s.q_meas) for s in states], axis=0)
        self.r.check(
            "far 0.3 m: gap_clamped fires (q_ik farther than the gap from q_meas)",
            bool(np.any(gap_count > 0)),
            f"messages gap-clamped per joint {gap_count.astype(int).tolist()} of {len(states)}; "
            f"max |q_ik - q_meas| {fmt(beyond)} rad",
        )
        self.r.check(
            "far 0.3 m: step_clamped fires",
            bool(np.any(step_count > 0)),
            f"messages step-clamped per joint {step_count.astype(int).tolist()} of {len(states)}",
        )
        self._bounds("far 0.3 m", states)
        mm, deg = pose_error(self.kin.fk(self.node.q()), target)
        # Counted in updates: from the first that moved the output to the first whose q_meas is
        # within 1 mm and 0.5 deg. At max_joint_step_rad per update that takes at least
        # max|dq| / step of them.
        moved = next((i for i, s in enumerate(states) if np.any(s.q_out != s.q_meas)), None)
        there = self._first_within(states, target)
        ok = moved is not None and there is not None
        updates = there - moved if ok else -1
        bound = (
            math.ceil(float(np.max(np.abs(states[there].q_meas - q) / self.max_step))) if ok else 0
        )
        self.r.check(
            "far 0.3 m: reached in the end, no faster than the step clamp allows",
            ok and mm < REACH_MM and deg < REACH_DEG and updates >= bound,
            f"{mm:.2f} mm / {deg:.2f} deg after 4 s; within 1 mm / 0.5 deg after {updates} "
            f"updates ({(states[there].t - t0) * 1000 if ok else float('nan'):.0f} ms); "
            f"the step clamp allows no fewer than {bound}",
        )
        self.node.stream(p0, 4.0)

    def stale_and_resume(self, p0: pin.SE3) -> None:
        timeout = self.values["target_timeout_s"]
        # The arm is moving when the targets stop, so the fallback has something to stop.
        mark = self.node.mark()
        last = self.node.stream(offset(p0, (0.0, 0.0, 0.03)), 0.12)
        time.sleep(0.6)
        states, joints, logs = self.node.since(mark)
        after = [s for s in states if s.t > last]
        first_stale = next((s.t - last for s in after if s.stale), float("nan"))
        late = [s for s in after if s.t > last + timeout + 0.05]
        settle = [j for j in joints if j[0] > last + timeout + 0.05]
        self.r.check(
            "stale: stopping the targets sets stale=1 after target_timeout_s",
            0.8 * timeout <= first_stale <= timeout + 0.05 and late and all(s.stale for s in late),
            f"first stale message {first_stale * 1000:.0f} ms after the last target "
            f"(timeout {timeout * 1000:.0f} ms); {len(late)} later messages all stale",
        )
        self.r.check(
            "stale: the output returns to the measured position, the arm stops",
            all(np.array_equal(s.q_out, s.q_meas) for s in late) and stationary(settle) == 0.0,
            f"q_out = q_meas in {sum(np.array_equal(s.q_out, s.q_meas) for s in late)}/"
            f"{len(late)}; joint change after stale {stationary(settle):.3g} rad",
        )
        self.r.check(
            "stale: throttled warning on /rosout",
            any("No target for" in m for _, m in logs),
            next((m for _, m in logs if "No target" in m), "none"),
        )
        self._bounds("stale fallback", after)
        mark = self.node.mark()
        t0 = time.monotonic()
        self.reach("resume after stale", p0, 2.0)
        states, _, _ = self.node.since(mark)
        self.restart_check("resume", states, t0, p0)

    def two_publishers(self, p0: pin.SE3) -> None:
        """crisp ignores /lbr/target_pose while it has two publishers; so must the passthrough."""
        second = self.node.create_publisher(PoseStamped, f"{NS}/target_pose", 1)
        time.sleep(1.0)  # discovery
        target = offset(p0, (0.0, 0.02, 0.0))
        q_before = self.node.q()
        mark = self.node.mark()
        self.node.stream(target, 1.0)
        time.sleep(0.05)
        states, joints, logs = self.node.since(mark)
        moved = max(stationary(joints), float(np.max(np.abs(self.node.q() - q_before))))
        held = bool(states) and all(s.stale and np.array_equal(s.q_out, s.q_meas) for s in states)
        self.r.check(
            "two publishers: the targets are ignored, the output stays measured",
            held and moved == 0.0,
            f"{len(states)} messages, all stale with q_out = q_meas: {held}; "
            f"largest joint change {moved:.3g} rad while a target 20 mm away streamed",
        )
        self.r.check(
            "two publishers: throttled warning on /rosout",
            any("Ignoring targets" in m for _, m in logs),
            next((m for _, m in logs if "Ignoring targets" in m), "none"),
        )
        self.node.destroy_publisher(second)
        time.sleep(1.0)
        self.reach("one publisher again", target, 2.0)
        self.node.stream(p0, 2.0)

    def restart_check(self, label: str, states: list[State], t0: float, target) -> None:
        fresh = next((s for s in states if not s.stale), None)
        # The IK restarts from the measured position with its filter on the target itself, so
        # q_ik is one update of the law from q_meas toward the raw target.
        predicted = (
            self.kin.ik_step(fresh.q_meas, target, self.posture, self.values) if fresh else None
        )
        error = float(np.max(np.abs(fresh.q_ik - predicted))) if fresh else float("nan")
        self.r.check(
            f"{label}: stale clears on the first new target, IK restarts at q_meas",
            fresh is not None and fresh.t - t0 < 0.1 and error < 1e-9,
            f"stale=0 {((fresh.t - t0) * 1000) if fresh else float('nan'):.0f} ms after the "
            f"first target; its q_ik is one law step from that q_meas to {error:.2g} rad "
            f"(the step: {fmt(np.abs(fresh.q_ik - fresh.q_meas)) if fresh else 'n/a'})",
        )


def main() -> int:
    problem = environment_problem()
    if problem:
        print(f"REFUSED: {problem}. This check runs on the loopback mock only.", file=sys.stderr)
        return 2

    rclpy.init()
    node = Mock()
    results = Results()
    try:
        with spinning(node):
            gate = Gate(node, results)
            if not gate.setup():
                return 2
            try:
                gate.run()
            finally:
                if node.cell() != "REST":
                    node.try_switch(activate=REST, deactivate=ARMED)
                print(f"\nleft at {node.cell()}")
    finally:
        rclpy.try_shutdown()
    return results.summary()


if __name__ == "__main__":
    sys.exit(main())
