"""Bench check of the passthrough's position_source modes on the mock stack.

It drives the plugin as the controller manager runs it: the parameters set at REST with
scripts/position_source.py, the mode taking effect at the next arm, the /lbr/target_pose
subscription, the stale fallback and ~/state.

    ROS_DOMAIN_ID=77 pixi run -e jazzy mock-loopback     # terminal 1
    pixi run -e jazzy check-mock                         # terminal 2

    pixi run -e jazzy mock-offset                        # terminal 1, in place of mock-loopback
    pixi run -e jazzy check-mock --offset-lagged         # terminal 2

mock_components/GenericSystem copies each position command into the position state at the next
read, so q_meas(k) = q_out(k-1):

* under ik_target the arm follows the passthrough's output, and FK of /lbr/joint_states is where
  the IK went;
* |q_out - q_meas| is one update's step: gap_clamped fires, but the gap cannot bind for longer
  than an update;
* lagged has nothing to lag. The offset mock makes q_meas(k) = q_out(k-1) + d, and the response
  to that step of d every update gives a = 1 - exp(-dt / tau).

Refuses to run unless ROS_DOMAIN_ID is 77, the RMW is CycloneDDS on this repository's loopback
file, and the robot_description is mock_components/GenericSystem. Leaves the stack at REST with
position_source measured. Exits 0 when every check passed.
"""

import argparse
import itertools
import math
import os
import re
import subprocess
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
    cell_state,
    controller_states,
    spinning,
    switch,
)
from position_source import PARAMETERS, get_parameters, spring_gap
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectoryPoint

REPO = Path(__file__).resolve().parents[1]
LOOPBACK = REPO / "config" / "cyclonedds_loopback.xml"
TOOL = REPO / "scripts" / "position_source.py"
N = len(JOINTS)
TARGET_RATE_HZ = 100.0
DT = 0.002  # controller_manager's update_rate is 500 (config/controllers.yaml)
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
        self.urdf: str | None = None
        self.create_subscription(
            Float64MultiArray,
            f"{NS}/{PASSTHROUGH}/state",
            self._on_state,
            QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(JointState, f"{NS}/joint_states", self._on_joints, 100)
        self.create_subscription(
            String,
            f"{NS}/robot_description",
            self._on_urdf,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
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

    def _on_urdf(self, message: String) -> None:
        self.urdf = message.data

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
        """One update of the ik_target law (docs/position_source.md) from q, target unfiltered."""
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


def consecutive(states: list[State], d: float = 0.0) -> list[tuple[State, State]]:
    """Pairs of ~/state messages from successive updates: on the mock q_meas(k) = q_out(k-1) + d.

    The realtime publisher skips a message when it cannot take its lock, which breaks the
    identity: a per-update check uses only the pairs that hold it.
    """
    pairs = []
    for a, b in itertools.pairwise(states):
        if np.all(np.abs(b.q_meas - (a.q_out + d)) <= 1e-12 + 1e-9 * abs(d)):
            moved = np.any(b.q_out != a.q_out)
            if d != 0.0 or moved:
                pairs.append((a, b))
    return pairs


def fmt(values) -> str:
    return "[" + ", ".join(f"{v:.4g}" for v in values) + "]"


def stationary(joints: list[tuple[float, np.ndarray]]) -> float:
    """Largest change of any joint over the recording, rad."""
    if len(joints) < 2:
        return float("nan")
    q = np.array([j[1] for j in joints])
    return float(np.max(np.abs(q - q[0])))


def run_tool(*args: str) -> tuple[int, str]:
    out = subprocess.run(
        [sys.executable, str(TOOL), *args], capture_output=True, text=True, timeout=60, check=False
    )
    text = (out.stdout + out.stderr).strip()
    # CycloneDDS warns about multicast on the loopback interface in every process.
    lines = [line for line in text.splitlines() if "multicast" not in line]
    return out.returncode, "\n".join(lines)


class Gate:
    def __init__(self, node: Mock, results: Results) -> None:
        self.node = node
        self.r = results

    def setup(self, home: bool = True) -> bool:
        deadline = time.monotonic() + 10.0
        while (self.node.urdf is None or not self.node.joints) and time.monotonic() < deadline:
            time.sleep(0.05)
        if self.node.urdf is None or not self.node.joints:
            print("REFUSED: no /lbr/robot_description or /lbr/joint_states: is the mock up?")
            return False
        if "mock_components/GenericSystem" not in self.node.urdf:
            print("REFUSED: the robot_description is not mock_components/GenericSystem")
            return False
        if self.node.cell() != "REST":
            print(f"REFUSED: not at REST ({self.node.cell()})")
            return False
        self.values = get_parameters(self.node, PASSTHROUGH, PARAMETERS)
        self.gap = np.array(spring_gap(self.values))
        self.max_step = np.array(self.values["max_joint_step_rad"])
        self.kin = Kinematics(self.node.urdf, self.values["end_effector_frame"])
        self.posture = self.node.q()
        print(f"gap {fmt(self.gap)} rad, step {fmt(self.max_step)} rad per update")
        if home:
            ok = self.node.move_joints(np.radians(HOME_DEGREES), 4.0)
            time.sleep(0.3)
            self.r.check(
                "setup: home through joint_trajectory_controller at REST",
                ok and np.allclose(self.node.q(), np.radians(HOME_DEGREES), atol=1e-3),
                f"q = {fmt(np.degrees(self.node.q()))} deg",
            )
        return True

    def set_mode(self, mode: str, *params: str) -> bool:
        args = ["set", mode, *[f"--param={p}" for p in params]]
        code, text = run_tool(*args)
        return self.r.check(
            f"{mode}: position_source.py {' '.join(args[1:])} at REST",
            code == 0 and "read back: identical" in text,
            f"exit {code}; {text.splitlines()[-1] if text else ''}",
        )

    def arm(self, mode: str) -> bool:
        before = self.node.cell()
        refused = self.node.try_switch(activate=ARMED, deactivate=REST)
        after = self.node.cell()
        return self.r.check(
            f"{mode}: REST -> ARMED (STRICT switch)",
            not refused and after.startswith("ARMED"),
            f"{before} -> {after}{'; ' + refused if refused else ''}",
        )

    def rest(self, mode: str) -> bool:
        mark = self.node.mark()
        refused = self.node.try_switch(activate=REST, deactivate=ARMED)
        time.sleep(0.2)
        _, _, logs = self.node.since(mark)
        # The passthrough's on_deactivate writes the measured position into the command for the
        # trajectory controller to hold, and logs an error when it could not.
        failed = [m for _, m in logs if "Deactivating without" in m or "Failed to leave" in m]
        return self.r.check(
            f"{mode}: ARMED -> REST, the measured position left in the command",
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

    def running_mode(self, mode: str) -> None:
        code, text = run_tool("status")
        line = next((x for x in text.splitlines() if x.startswith("running")), "")
        self.r.check(
            f"{mode}: position_source.py status reads the running mode",
            code == 0 and line.startswith(f"running (~/state): {mode}"),
            line or text.splitlines()[-1],
        )

    def refuses_limit_changes(self) -> None:
        # 20 for 200 would be a 0.5 rad gap.
        names = ("position_source", "sunrise_joint_stiffness")
        before = get_parameters(self.node, PASSTHROUGH, names)
        code, text = run_tool("set", "ik_target", "--param=sunrise_joint_stiffness=20")
        after = get_parameters(self.node, PASSTHROUGH, names)
        self.r.check(
            "tool: `set` refuses a looser spring-torque limit without --limit-change-reviewed",
            code == 1 and "--limit-change-reviewed" in text and after == before,
            f"exit {code}; " + next((x for x in text.splitlines() if "REFUSED" in x), text)[:120],
        )

    def per_joint_step(self) -> None:
        steps = [0.004, 0.004, 0.004, 0.004, 0.003, 0.003, 0.002]
        ok = self.set_mode("lagged", "max_joint_step_rad=" + ",".join(map(str, steps)))
        live = get_parameters(self.node, PASSTHROUGH, ("max_joint_step_rad",))["max_joint_step_rad"]
        self.r.check(
            "tool: a per-joint max_joint_step_rad sets as a double array",
            ok and live == steps and all(isinstance(v, float) for v in live),
            f"live {live}",
        )
        self.set_mode("lagged", f"max_joint_step_rad={self.max_step[0]}")

    def refuses_when_armed(self) -> None:
        code, text = run_tool("set", "measured")
        self.r.check(
            "tool: `set` refuses while ARMED",
            code == 1 and "REFUSED" in text,
            text.splitlines()[0] if text else f"exit {code}",
        )

    def passthrough_holds(self, label: str, mode: int, seconds: float, target=None) -> None:
        """measured and lagged on the mock: the output is the measured position, the arm stays."""
        q_before = self.node.q()
        mark = self.node.mark()
        if target is not None:
            self.node.stream(target, seconds)
        else:
            time.sleep(seconds)
        states, joints, _ = self.node.since(mark)
        n = len(states)
        modes = {s.mode for s in states}
        identical = all(
            np.array_equal(s.q_out, s.q_meas) and s.q_out.tobytes() == s.q_meas.tobytes()
            for s in states
        )
        flags = sum(int(s.gap.any() or s.step.any() or s.stale) for s in states)
        rate = n / seconds
        moved = max(stationary(joints), float(np.max(np.abs(self.node.q() - q_before))))
        what = "while a target 5 cm away streams" if target is not None else "with no target"
        self.r.check(
            f"{label}: q_out bit-identical to q_meas {what}",
            n > 0.5 * seconds / DT and modes == {mode} and identical and flags == 0,
            f"{n} ~/state messages ({rate:.0f}/s), mode {sorted(modes)}, "
            f"{n if identical else 'NOT all'} identical (memcmp), {flags} with a flag or stale",
        )
        self.r.check(
            f"{label}: the arm stays put {what}",
            moved == 0.0,
            f"largest joint change {moved:.3g} rad over {seconds:.1f} s",
        )

    def holds(self, mode: str, index: int) -> None:
        self.running_mode(mode)
        if mode == "measured":
            self.refuses_when_armed()
        self.passthrough_holds(mode, index, 1.0)
        here = self.kin.fk(self.node.q())
        self.passthrough_holds(mode, index, 1.0, target=offset(here, (0.05, 0.0, 0.0)))
        self.rest(mode)

    def measured(self) -> None:
        print("\n== measured")
        self.refuses_limit_changes()
        self.per_joint_step()
        if not self.set_mode("measured"):
            return
        q_rest = self.node.q()
        mark = self.node.mark()
        if not self.arm("measured"):
            return
        time.sleep(0.2)
        states, _, logs = self.node.since(mark)
        first = states[0] if states else None
        self.r.check(
            "measured: first output after activation = the measured position at REST",
            first is not None and np.array_equal(first.q_out, q_rest),
            f"|q_out - q_rest| max {np.max(np.abs(first.q_out - q_rest)) if first else 'n/a'}",
        )
        self.logged("measured: activation logs the mode", logs, "position_source: measured.")
        self.holds("measured", 0)

    def lagged(self) -> None:
        print("\n== lagged (stock mock: nothing to lag; the offset run measures tau)")
        if not self.set_mode("lagged"):
            return
        mark = self.node.mark()
        if not self.arm("lagged"):
            return
        time.sleep(0.2)
        _, _, logs = self.node.since(mark)
        self.logged("lagged: activation logs the mode", logs, "position_source: lagged")
        self.holds("lagged", 1)

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

    def ik_target(self) -> None:
        print("\n== ik_target")
        if not self.set_mode("ik_target"):
            return
        here0 = self.kin.fk(self.node.q())
        self.node.stream(offset(here0, (0.0, 0.0, 0.05)), 0.3)
        time.sleep(0.2)
        self.ik_arm_ignores_old_target("ik_target (first arm)")

        p0 = self.kin.fk(self.node.q())
        targets = [
            ("+30 mm x", offset(p0, (0.03, 0.0, 0.0))),
            ("+30 mm x, -20 mm z, 10 deg about x", offset(p0, (0.03, 0.0, -0.02), (10, 0, 0))),
            ("-20 mm y, +15 mm z, 5 deg y, -8 deg z", offset(p0, (0.0, -0.02, 0.015), (0, 5, -8))),
            ("back to the start pose", p0),
        ]
        for name, pose in targets:
            self.reach(f"ik_target {name}", pose, 2.0)

        self.far(p0)
        self.stale_and_resume(p0)
        self.two_publishers(p0)
        self.running_mode("ik_target")

        if not self.rest("ik_target"):
            return
        here = self.kin.fk(self.node.q())
        self.node.stream(offset(here, (0.0, 0.05, 0.0)), 0.5)
        time.sleep(0.2)
        self.ik_arm_ignores_old_target("ik_target (re-arm)")
        here = self.kin.fk(self.node.q())
        target = offset(here, (0.0, 0.0, 0.01))
        mark = self.node.mark()
        t0 = time.monotonic()
        self.reach("ik_target (re-arm) +10 mm z", target, 1.5)
        self.restart_check("ik_target (re-arm)", self.node.since(mark)[0], t0, target)
        self.rest("ik_target")

    def ik_arm_ignores_old_target(self, label: str) -> None:
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
            self.r.check("ik_target far: a reachable 0.3 m target", False, "none found")
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
            "ik_target far 0.3 m: gap_clamped fires (q_ik farther than the gap from q_meas)",
            bool(np.any(gap_count > 0)),
            f"messages gap-clamped per joint {gap_count.astype(int).tolist()} of {len(states)}; "
            f"max |q_ik - q_meas| {fmt(beyond)} rad",
        )
        self.r.check(
            "ik_target far 0.3 m: step_clamped fires",
            bool(np.any(step_count > 0)),
            f"messages step-clamped per joint {step_count.astype(int).tolist()} of {len(states)}",
        )
        self._bounds("ik_target far 0.3 m", states)
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
            "ik_target far 0.3 m: reached in the end, no faster than the step clamp allows",
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
            "ik_target stale: stopping the targets sets stale=1 after target_timeout_s",
            0.8 * timeout <= first_stale <= timeout + 0.05 and late and all(s.stale for s in late),
            f"first stale message {first_stale * 1000:.0f} ms after the last target "
            f"(timeout {timeout * 1000:.0f} ms); {len(late)} later messages all stale",
        )
        self.r.check(
            "ik_target stale: the output returns to the measured position, the arm stops",
            all(np.array_equal(s.q_out, s.q_meas) for s in late) and stationary(settle) == 0.0,
            f"q_out = q_meas in {sum(np.array_equal(s.q_out, s.q_meas) for s in late)}/"
            f"{len(late)}; joint change after stale {stationary(settle):.3g} rad",
        )
        self.r.check(
            "ik_target stale: throttled warning on /rosout",
            any("No target for" in m for _, m in logs),
            next((m for _, m in logs if "No target" in m), "none"),
        )
        self._bounds("ik_target stale fallback", after)
        mark = self.node.mark()
        t0 = time.monotonic()
        self.reach("ik_target resume after stale", p0, 2.0)
        states, _, _ = self.node.since(mark)
        self.restart_check("ik_target resume", states, t0, p0)

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
            "ik_target two publishers: the targets are ignored, the output stays measured",
            held and moved == 0.0,
            f"{len(states)} messages, all stale with q_out = q_meas: {held}; "
            f"largest joint change {moved:.3g} rad while a target 20 mm away streamed",
        )
        self.r.check(
            "ik_target two publishers: throttled warning on /rosout",
            any("Ignoring targets" in m for _, m in logs),
            next((m for _, m in logs if "Ignoring targets" in m), "none"),
        )
        self.node.destroy_publisher(second)
        time.sleep(1.0)
        self.reach("ik_target one publisher again", target, 2.0)
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

    def offset_lagged(self, taus: list[float]) -> bool:
        match = re.search(
            r'name="position_state_following_offset">\s*([-+0-9.eE]+)\s*<', self.node.urdf
        )
        if not match:
            print(
                "REFUSED: the robot_description has no position_state_following_offset: start "
                "`pixi run -e jazzy mock-offset`"
            )
            return False
        d = float(match.group(1))
        print(f"\n== lagged, offset mock: q_meas(k) = q_out(k-1) + {d}")
        # measured copies q_meas, so on this mock the arm drifts d per update.
        if self.set_mode("measured") and self.arm("measured (offset)"):
            mark = self.node.mark()
            time.sleep(0.3)
            states, _, _ = self.node.since(mark)
            self.rest("measured (offset)")
            pairs = consecutive(states, d)
            self.r.check(
                "measured (offset): q_out = q_meas exactly while q_meas steps d every update",
                len(pairs) > 50 and all(np.array_equal(s.q_out, s.q_meas) for s in states),
                f"{len(pairs)} successive updates, every message identical",
            )
        for tau in taus:
            if not (
                self.set_mode("lagged", f"lag_time_constant_s={tau}")
                and self.arm(f"lagged tau {tau}")
            ):
                continue
            mark = self.node.mark()
            time.sleep(0.3)
            states, _, _ = self.node.since(mark)
            self.rest(f"lagged tau {tau}")
            pairs = consecutive(states, d)
            # Each update q_meas steps by d off the last output; the low-pass takes a of it.
            a = np.array([(b.q_out - a_.q_out) / (b.q_meas - a_.q_out) for a_, b in pairs])
            if a.size == 0:
                self.r.check(f"lagged tau {tau}: per-update response", False, "no successive pairs")
                continue
            a_med = float(np.median(a))
            tau_est = -DT / math.log(1 - a_med) if 0 < a_med < 1 else float("nan")
            expected = -math.expm1(-DT / tau)
            ticks = math.log(1 / math.e) / math.log(1 - a_med) if 0 < a_med < 1 else float("nan")
            # a uses each update's own period, which jitters around 2 ms on a kernel that is not
            # realtime.
            lo, hi = np.percentile(a, [5, 95])
            self.r.check(
                f"lagged tau {tau}: per-update fraction a = 1 - exp(-dt/tau) of the measured step",
                abs(a_med - expected) < 0.02 * expected
                and expected * 0.9 < lo <= hi < expected * 1.1,
                f"a median {a_med:.5f} (p5-p95 {lo:.5f}-{hi:.5f}) vs {expected:.5f} at dt 2 ms, "
                f"over {len(pairs)} updates x 7 joints -> tau {tau_est * 1000:.2f} ms, 63 % of a "
                f"step in {ticks:.2f} updates = {ticks * DT * 1000:.1f} ms",
            )
            flags = sum(int(s.gap.any() or s.step.any()) for s in states)
            gap = np.max([np.abs(s.q_out - s.q_meas) for s in states], axis=0)
            self.r.check(
                f"lagged tau {tau}: q_meas - q_out settles at (1 - a) d, no clamp",
                flags == 0 and np.allclose(gap, (1 - expected) * d, rtol=0.05),
                f"max |q_out - q_meas| {fmt(gap)} vs (1-a)d = {(1 - expected) * d:.4g}; "
                f"{flags} messages with a clamp",
            )
        return True


def run(node: Mock, results: Results, args: argparse.Namespace) -> bool:
    gate = Gate(node, results)
    if not gate.setup(home=not args.offset_lagged):
        return False
    try:
        if args.offset_lagged:
            return gate.offset_lagged(args.taus)
        gate.measured()
        gate.lagged()
        gate.ik_target()
        return True
    finally:
        if node.cell() != "REST":
            node.try_switch(activate=REST, deactivate=ARMED)
        code, _ = run_tool("set", "measured")
        print(f"\nleft at {node.cell()}, set measured: exit {code}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--offset-lagged",
        action="store_true",
        help="only the lagged time-constant check, on the offset mock",
    )
    parser.add_argument("--taus", type=float, nargs="+", default=[0.008, 0.02, 0.05])
    args = parser.parse_args()

    problem = environment_problem()
    if problem:
        print(f"REFUSED: {problem}. This check runs on the loopback mock only.", file=sys.stderr)
        return 2

    rclpy.init()
    node = Mock()
    results = Results()
    try:
        with spinning(node):
            if not run(node, results, args):
                return 2
    finally:
        rclpy.try_shutdown()
    return results.summary()


if __name__ == "__main__":
    sys.exit(main())
