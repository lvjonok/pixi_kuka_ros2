"""Record a teleoperated camera trajectory, replay it under candidate gains, score the tracking.

Tuning the Cartesian impedance by feel mixes the gains with the operator: every try is a
different motion. This fixes the motion. `record` captures what the arm was actually sent
(/lbr/target_pose, after every clamp upstream of it) and what it did (/lbr/current_pose,
/lbr/joint_states) while someone teleoperates. `replay` sends that same target sequence again,
once per gains file, and scores each run against the same numbers:

* lag -- the time shift that best aligns the arm with the target, and the error left after it
  (the shape error: overshoot, ringing, anything a pure delay does not explain);
* position and rotation error, RMS and max, against the target the controller held;
* the peak joint speed as a fraction of the joint's limit -- lbr_fri_ros2's CommandGuard drops
  FRI above 1.0 (26 Sep 2026: A1 during Haply teleop with k_rot 100).

Why a lag exists at all: crisp damps against MEASURED velocity with the target's velocity
taken as zero, so a target moving at v holds a steady-state error of (D/K) v behind it -- 65 ms
at k_pos 1300 / d_pos 84. A gains file may set `replay: {lead: true}` to send each target
(D/K) v ahead of itself (camera_gizmo.py's lead), which cancels that at constant speed, and
`replay: {max_error_m: 0.04}` to hold the target within that of the arm (crisp's
max_ee_tracking_error, which a replay otherwise bypasses).

The first recording (26 Sep 2026, baseline gains, cap 0.2 m) ended in a CommandGuard stop:
the target never exceeded 0.24 m/s over 100 ms, but the arm fell 40-60 mm behind it and then
surged to catch up, and A4 (75 deg/s, the lowest limit) reached 123 %. `arm_peak_mps` against
`target_peak_mps` is that surge. The guard then latches: on a violation it returns without
updating its previous position (command_guard.cpp), so every later sample fails against it.

    # 1. record, while the operator teleoperates (lerobot_pickplace `make teleop-arm`):
    pixi run -e jazzy python scripts/track_tune.py record ~/data/track/rec1.jsonl
    # 2. arm the cell (lerobot_pickplace scripts/kuka_cell.py arm), then:
    pixi run -e jazzy python scripts/track_tune.py replay ~/data/track/rec1.jsonl \
        --gains g/baseline.yaml g/try1.yaml --restore g/baseline.yaml --out ~/data/track/sweep1

Gains files are `ros2 param load` files for /lbr/cartesian_impedance_controller (nested keys
are flattened with dots). Every parameter is dynamic in crisp_controllers; they are set over
the node's set_parameters service before each run, and --restore is set on every way out.

replay MOVES THE ARM, in T1 with the enabling switch held. Before each run it walks the target
from where the arm stands to the recording's first pose (5 cm/s, 15 deg/s) and settles. It
refuses unless cartesian_impedance_controller is active and nothing else publishes targets, and
it aborts the run -- holds the measured pose, keeps the partial log, goes on to the next file -- on a tracking error above
--abort-m / --abort-deg, a joint above --abort-speed of its limit, a target under --floor-z, a
stale pose, or a second publisher. It publishes directly, not through crisp_py, whose Robot
starts its own target publisher on construction.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import pinocchio as pin
import rclpy
import rclpy.signals
import yaml
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import JointState
from std_msgs.msg import String

NS = "/lbr"
TARGET_TOPIC = f"{NS}/target_pose"
CARTESIAN = "cartesian_impedance_controller"
BASE_FRAME = "lbr_link_0"
JOINT_NAMES = tuple(f"lbr_A{i + 1}" for i in range(7))
RATE_HZ = 100.0
# Joined after rclpy shuts down: a spin thread still running when the process exits segfaults
# (or aborts, "terminate called without an active exception").
_CELLS: list[Cell] = []


# -- recording file ------------------------------------------------------------------------


def _pose_row(t: float, kind: str, msg: PoseStamped) -> dict[str, Any]:
    p, o = msg.pose.position, msg.pose.orientation
    return {"t": t, "k": kind, "p": [p.x, p.y, p.z], "q": [o.x, o.y, o.z, o.w]}


def load_series(path: Path, kind: str) -> tuple[np.ndarray, np.ndarray, Rotation]:
    """``(t, positions, rotations)`` of one pose stream in a recording, t from its first row."""
    rows = [r for r in map(json.loads, path.read_text().splitlines()) if r["k"] == kind]
    if len(rows) < 2:
        raise SystemExit(f"{path}: fewer than two {kind!r} rows")
    t = np.array([r["t"] for r in rows])
    return t, np.array([r["p"] for r in rows]), Rotation.from_quat([r["q"] for r in rows])


# -- node ----------------------------------------------------------------------------------


class Cell(Node):
    """Topics and services of the running cell; state updated by a background executor."""

    def __init__(self, *, publish: bool) -> None:
        super().__init__("track_tune")
        self.lock = threading.Lock()
        self.pose: tuple[float, np.ndarray, Rotation] | None = None
        self.q: np.ndarray | None = None
        self.dq: np.ndarray | None = None
        self.sink: Any = None  # callable(row) while recording
        # lbr_fri_ros2's CommandGuard differentiates MEASURED positions over one FRI sample; the
        # reported joint_states velocity is smoother and read 1.01 where that read 1.23 (26 Sep).
        # The guard here uses the larger of the two, the difference taken over ~10 ms.
        self.q_hist: list[tuple[float, np.ndarray]] = []
        self.create_subscription(
            PoseStamped, f"{NS}/current_pose", self._on_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, f"{NS}/joint_states", self._on_joints, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(PoseStamped, TARGET_TOPIC, 1) if publish else None
        if not publish:
            self.create_subscription(PoseStamped, TARGET_TOPIC, self._on_target, 10)
            # The estimated contact wrench (tweezer tips, in lbr_umi_camera's axes), so a
            # recording can say what the estimator reads in free motion -- the phantom force the
            # Haply renders as walls.
            self.create_subscription(
                WrenchStamped, f"{NS}/force_torque_broadcaster/wrench", self._on_wrench,
                qos_profile_sensor_data,
            )
        self.executor_ = SingleThreadedExecutor()
        self.executor_.add_node(self)
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        _CELLS.append(self)
        self.spinner.start()

    def _spin(self) -> None:
        try:
            self.executor_.spin()
        except (ExternalShutdownException, rclpy.executors.ShutdownException):
            pass

    def _on_pose(self, msg: PoseStamped) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        now = time.monotonic()
        with self.lock:
            self.pose = (now, np.array([p.x, p.y, p.z]), Rotation.from_quat([o.x, o.y, o.z, o.w]))
        if self.sink:
            self.sink(_pose_row(now, "pose", msg))

    def _on_target(self, msg: PoseStamped) -> None:
        if self.sink:
            self.sink(_pose_row(time.monotonic(), "target", msg))

    def _on_wrench(self, msg: WrenchStamped) -> None:
        if self.sink:
            f, m = msg.wrench.force, msg.wrench.torque
            self.sink({"t": time.monotonic(), "k": "wrench", "frame": msg.header.frame_id,
                       "f": [f.x, f.y, f.z], "m": [m.x, m.y, m.z]})

    def _on_joints(self, msg: JointState) -> None:
        pos, vel = dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity))
        if not all(n in pos for n in JOINT_NAMES):
            return
        now = time.monotonic()
        q = np.array([pos[n] for n in JOINT_NAMES])
        reported = np.abs([vel.get(n, 0.0) for n in JOINT_NAMES])
        with self.lock:
            self.q_hist = [*self.q_hist[-5:], (now, q)]
            t_old, q_old = self.q_hist[0]
            fd = np.abs(q - q_old) / (now - t_old) if now - t_old > 1e-3 else np.zeros(7)
            self.q, self.dq = q, np.maximum(reported, fd)
        if self.sink:
            self.sink({"t": time.monotonic(), "k": "joints", "q": self.q.tolist(),
                       "dq": self.dq.tolist()})

    def call(self, client: Any, request: Any, timeout_s: float = 5.0) -> Any:
        """A service call answered by the background executor."""
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise SystemExit(f"service {client.srv_name} is not up")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                raise SystemExit(f"service {client.srv_name} did not answer in {timeout_s} s")
            time.sleep(0.01)
        return future.result()

    def active_controllers(self) -> set[str]:
        client = self.create_client(ListControllers, f"{NS}/controller_manager/list_controllers")
        result = self.call(client, ListControllers.Request())
        return {c.name for c in result.controller if c.state == "active"}

    def get_gains(self, names: list[str]) -> dict[str, float]:
        client = self.create_client(GetParameters, f"{NS}/{CARTESIAN}/get_parameters")
        result = self.call(client, GetParameters.Request(names=names))
        return {n: v.double_value for n, v in zip(names, result.values)}

    def set_gains(self, gains: dict[str, float]) -> None:
        client = self.create_client(SetParameters, f"{NS}/{CARTESIAN}/set_parameters")
        params = [
            Parameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)),
            )
            for name, v in gains.items()
        ]
        result = self.call(client, SetParameters.Request(parameters=params))
        refused = [(p.name, r.reason) for p, r in zip(params, result.results) if not r.successful]
        if refused:
            raise SystemExit(f"{CARTESIAN} refused {refused}")

    def publish(self, p: np.ndarray, r: Rotation) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in p)
        x, y, z, w = r.as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
        self.pub.publish(msg)

    def velocity_limits(self) -> np.ndarray:
        """Per-joint velocity limits, rad/s, from the robot_description the stack runs."""
        got: list[str] = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.create_subscription(
            String, f"{NS}/robot_description", lambda m: got.append(m.data), qos
        )
        deadline = time.monotonic() + 10.0
        while not got and time.monotonic() < deadline:
            time.sleep(0.05)
        self.destroy_subscription(sub)
        if not got:
            raise SystemExit(f"no {NS}/robot_description within 10 s -- is the stack up?")
        model = pin.buildModelFromXML(got[0])
        return np.array(
            [model.velocityLimit[model.joints[model.getJointId(n)].idx_v] for n in JOINT_NAMES]
        )


# -- record --------------------------------------------------------------------------------


def record(args: argparse.Namespace) -> int:
    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    cell = Cell(publish=False)
    counts = {"target": 0, "pose": 0, "joints": 0, "wrench": 0}
    lock = threading.Lock()
    with out.open("w") as f:

        def sink(row: dict[str, Any]) -> None:
            with lock:
                f.write(json.dumps(row) + "\n")
                counts[row["k"]] += 1

        cell.sink = sink
        print(f"recording {TARGET_TOPIC}, current_pose, joint_states -> {out}; ^C to stop",
              flush=True)
        try:
            while True:
                time.sleep(2.0)
                with lock:
                    print(f"  {counts}", flush=True)
        except KeyboardInterrupt:
            pass
        cell.sink = None
    print(f"wrote {out}: {counts}")
    if counts["target"] < 2:
        print("NO TARGETS: nothing published on /lbr/target_pose while recording", file=sys.stderr)
        return 1
    return 0


def steps(args: argparse.Namespace) -> int:
    """Write a recording of small steps about where the arm stands now. Reads, moves nothing.

    Each step: hold at the start pose, jump the target by a few mm (or a fraction of a degree)
    along one base axis, hold, jump back. `replay` then drives it under each gains file and
    `fine` scores the answer: the hold error left after each step (friction, or a torque the
    spring is fighting) and the delay to half the step.
    """
    cell = Cell(publish=False)
    deadline = time.monotonic() + 5.0
    while cell.pose is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if cell.pose is None:
        raise SystemExit("no /lbr/current_pose within 5 s -- is the cell up?")
    _, p0, r0 = cell.pose
    hold_s, dt = args.hold_s, 1.0 / RATE_HZ
    seq: list[tuple[np.ndarray, Rotation]] = []

    def hold(p: np.ndarray, r: Rotation) -> None:
        seq.extend([(p, r)] * int(hold_s * RATE_HZ))

    hold(p0, r0)
    for axis in range(3):
        for mm in args.mm:
            for sign in (1, -1):
                d = np.zeros(3)
                d[axis] = sign * mm * 1e-3
                hold(p0 + d, r0)
                hold(p0, r0)
    for axis in range(3):
        for deg in args.deg:
            for sign in (1, -1):
                v = np.zeros(3)
                v[axis] = np.radians(sign * deg)
                hold(p0, Rotation.from_rotvec(v) * r0)
                hold(p0, r0)
    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as f:
        for i, (p, r) in enumerate(seq):
            f.write(json.dumps({"t": i * dt, "k": "target", "p": p.tolist(),
                                "q": r.as_quat().tolist()}) + "\n")
    print(f"wrote {out}: {len(seq) * dt:.0f} s of steps ({args.mm} mm, {args.deg} deg, "
          f"hold {hold_s} s) about camera {np.round(p0, 4).tolist()}")
    return 0


# -- replay --------------------------------------------------------------------------------


def _flatten(tree: dict[str, Any], prefix: str = "") -> dict[str, float]:
    out: dict[str, float] = {}
    for k, v in tree.items():
        name = f"{prefix}{k}"
        if isinstance(v, dict):
            out |= _flatten(v, name + ".")
        else:
            out[name] = float(v)
    return out


def load_gains(path: Path) -> tuple[dict[str, float], dict[str, Any]]:
    """``(parameters, replay options)`` from a ros2-param-load file for the controller."""
    doc = yaml.safe_load(Path(path).expanduser().read_text())
    (node,) = doc.values()
    options = node.get("replay", {}) or {}
    return _flatten(node["ros__parameters"]), options


class Abort(Exception):
    """A safety condition; the arm is left holding its measured pose."""


def _guard(cell: Cell, args: argparse.Namespace, vlim: np.ndarray, p: np.ndarray,
           r: Rotation) -> tuple[np.ndarray, Rotation, np.ndarray]:
    with cell.lock:
        pose, dq = cell.pose, cell.dq
    if pose is None or time.monotonic() - pose[0] > 0.2:
        raise Abort("current_pose is stale (> 0.2 s)")
    if cell.count_publishers(TARGET_TOPIC) > 1:
        raise Abort("another node publishes on /lbr/target_pose")
    if p[2] < args.floor_z:
        raise Abort(f"target z {p[2]:.3f} under the floor {args.floor_z}")
    _, mp, mr = pose
    if np.linalg.norm(p - mp) > args.abort_m:
        raise Abort(f"tracking error {np.linalg.norm(p - mp) * 1e3:.0f} mm > {args.abort_m} m")
    if np.degrees((r * mr.inv()).magnitude()) > args.abort_deg:
        raise Abort(f"rotation error > {args.abort_deg} deg")
    if dq is not None and np.nanmax(np.abs(dq) / vlim) > args.abort_speed:
        j = int(np.nanargmax(np.abs(dq) / vlim))
        raise Abort(f"{JOINT_NAMES[j]} at {abs(dq[j]) / vlim[j]:.0%} of its velocity limit")
    return mp, mr, dq


def _approach(cell: Cell, args: argparse.Namespace, vlim: np.ndarray, p0: np.ndarray,
              r0: Rotation) -> None:
    """Walk the target from the measured pose to (p0, r0), then settle."""
    with cell.lock:
        _, sp, sr = cell.pose
    dist = float(np.linalg.norm(p0 - sp))
    ang = float((r0 * sr.inv()).magnitude())
    duration = max(dist / 0.05, np.degrees(ang) / 15.0, 0.5)
    slerp = Slerp([0.0, 1.0], Rotation.concatenate([sr, r0]))
    print(f"   approach: {dist * 1e3:.0f} mm, {np.degrees(ang):.1f} deg over {duration:.1f} s, "
          f"then settle {args.settle_s:.1f} s", flush=True)
    t0 = time.monotonic()
    while (s := (time.monotonic() - t0) / duration) < 1.0:
        s = 0.5 - 0.5 * np.cos(np.pi * s)
        p, r = sp + s * (p0 - sp), slerp([s])[0]
        _guard(cell, args, vlim, p, r)
        cell.publish(p, r)
        time.sleep(1.0 / RATE_HZ)
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.settle_s:
        _guard(cell, args, vlim, p0, r0)
        cell.publish(p0, r0)
        time.sleep(1.0 / RATE_HZ)


def _run(cell: Cell, args: argparse.Namespace, vlim: np.ndarray, t: np.ndarray, P: np.ndarray,
         R: Rotation, lead: tuple[float, float] | None,
         max_error_m: float | None, log: list[dict[str, Any]]) -> None:
    """Send the recorded targets on their own clock; append (sent, measured, dq) each tick."""
    t0 = time.monotonic()
    i = 0
    shown = -1
    while (el := time.monotonic() - t0) <= t[-1]:
        while i + 1 < len(t) and t[i + 1] <= el:
            i += 1
        p, r = P[i], R[i]
        if lead is not None and i > 0:
            dt = max(t[i] - t[i - 1], 1e-3)
            v = (P[i] - P[i - 1]) / dt
            w = (R[i] * R[i - 1].inv()).as_rotvec() / dt
            p = p + lead[0] * v
            r = Rotation.from_rotvec(lead[1] * w) * r
        if max_error_m is not None:
            # crisp's max_ee_tracking_error, which this replay bypasses otherwise.
            with cell.lock:
                _, mp0, _ = cell.pose
            gap = p - mp0
            if np.linalg.norm(gap) > max_error_m:
                p = mp0 + gap * (max_error_m / np.linalg.norm(gap))
        mp, mr, dq = _guard(cell, args, vlim, p, r)
        cell.publish(p, r)
        log.append({"t": el, "ref_p": P[i].tolist(), "ref_q": R[i].as_quat().tolist(),
                    "p": mp.tolist(), "q": mr.as_quat().tolist(),
                    "dq": None if dq is None else dq.tolist()})
        # Once a second, so a run of millimetre steps is visibly alive: where the target is
        # relative to the run's start, and how far the arm is from it.
        if int(el) != shown:
            shown = int(el)
            print(f"   {el:5.1f}/{t[-1]:.0f} s  target {np.round((P[i] - P[0]) * 1e3, 1)} mm "
                  f"{np.degrees((R[i] * R[0].inv()).magnitude()):.1f} deg  arm off by "
                  f"{np.linalg.norm(P[i] - mp) * 1e3:.1f} mm "
                  f"{np.degrees((R[i] * mr.inv()).magnitude()):.2f} deg", flush=True)
        time.sleep(1.0 / RATE_HZ)


def score(log: list[dict[str, Any]], vlim: np.ndarray) -> dict[str, float]:
    """Lag, error after the lag, raw error, and the joint-speed margin of one run."""
    t = np.array([row["t"] for row in log])
    ref_p = np.array([row["ref_p"] for row in log])
    ref_r = Rotation.from_quat([row["ref_q"] for row in log])
    mp = np.array([row["p"] for row in log])
    mr = Rotation.from_quat([row["q"] for row in log])

    e_p = np.linalg.norm(ref_p - mp, axis=1) * 1e3
    e_r = np.degrees((ref_r * mr.inv()).magnitude())

    # The shift that best explains the arm as a delayed copy of the target.
    grid = np.arange(0.0, 0.401, 0.005)
    tt = t[(t > grid[-1])]

    def shifted_rms(tau: float) -> tuple[float, float]:
        rp = np.stack([np.interp(tt - tau, t, ref_p[:, k]) for k in range(3)], axis=1)
        mpp = np.stack([np.interp(tt, t, mp[:, k]) for k in range(3)], axis=1)
        idx = np.clip(np.searchsorted(t, tt - tau), 0, len(t) - 1)
        jdx = np.clip(np.searchsorted(t, tt), 0, len(t) - 1)
        er = np.degrees((ref_r[idx] * mr[jdx].inv()).magnitude())
        return float(np.sqrt(np.mean(np.sum((rp - mpp) ** 2, axis=1))) * 1e3), float(
            np.sqrt(np.mean(er**2))
        )

    rows = [shifted_rms(tau) for tau in grid]
    kp, kr = int(np.argmin([a for a, _ in rows])), int(np.argmin([b for _, b in rows]))

    # Surge: the arm's fastest 100 ms against the target's. Above 1 the arm is catching up on
    # a lag faster than it was asked to move -- how A4 passed its limit on 26 Sep.
    def peak_speed(x: np.ndarray) -> float:
        j = np.searchsorted(t, t + 0.1)
        ok = j < len(t)
        return float(np.max(np.linalg.norm(x[j[ok]] - x[ok], axis=1) / (t[j[ok]] - t[ok])))
    dq = np.array([row["dq"] for row in log if row["dq"] is not None])
    speed = np.nanmax(np.abs(dq) / vlim, axis=0) if len(dq) else np.full(7, np.nan)
    return {
        "pos_rms_mm": float(np.sqrt(np.mean(e_p**2))),
        "pos_max_mm": float(e_p.max()),
        "pos_lag_ms": float(grid[kp] * 1e3),
        "pos_rms_after_lag_mm": rows[kp][0],
        "rot_rms_deg": float(np.sqrt(np.mean(e_r**2))),
        "rot_max_deg": float(e_r.max()),
        "rot_lag_ms": float(grid[kr] * 1e3),
        "rot_rms_after_lag_deg": rows[kr][1],
        "arm_peak_mps": peak_speed(mp),
        "target_peak_mps": peak_speed(ref_p),
        "peak_joint_speed_frac": float(np.nanmax(speed)),
        "peak_joint": JOINT_NAMES[int(np.nanargmax(speed))],
    } | fine(t, ref_p, ref_r, mp, mr)


# -- small motions -------------------------------------------------------------------------

FINE_HZ = 100.0
STILL_S = 0.3        # the target has not moved for this long: the arm should be on it
STILL_MM = 0.2       # ... by more than this
STILL_DEG = 0.05
HOLD_S = 1.0         # ... and for this long: the transient is over
SMALL_MM = (0.5, 10.0)


def fine(t: np.ndarray, ref_p: np.ndarray, ref_r: Rotation, mp: np.ndarray,
         mr: Rotation) -> dict[str, float]:
    """How the arm answers SMALL motions: what the lag score averages away.

    26 Sep 2026, first gripper session: "unresponsive in very small movements". Two numbers
    say which kind:

    * hold error -- where the arm sits once the target has been still for 1 s. Joint
      friction the spring cannot overcome leaves a residual of about friction / stiffness
      that no amount of waiting removes (p50 / p95, mm and deg).
    * small moves -- target moves of 0.5-10 mm between two stills: how much later than the
      target the arm covers half of it, and the error 0.5 s after the target stops.

    ``ref_*`` is the target the controller held at each ``t`` (zero-order hold of what was
    published), ``m*`` the measured pose at the same times.
    """
    g = np.arange(t[0], t[-1], 1.0 / FINE_HZ)
    i = np.clip(np.searchsorted(t, g, side="right") - 1, 0, len(t) - 1)
    rp, mpp, rr, mrr = ref_p[i], mp[i], ref_r[i], mr[i]
    back = int(STILL_S * FINE_HZ)
    # The target's largest excursion over the last STILL_S (a move has ended) and over the
    # last HOLD_S (the arm has had time to arrive: what is left is what it will not close).
    hold = int(HOLD_S * FINE_HZ)
    moved = np.zeros(len(g))
    turned = np.zeros(len(g))
    still = np.zeros(len(g), dtype=bool)
    for k in range(1, hold + 1):
        moved[k:] = np.maximum(moved[k:], np.linalg.norm(rp[k:] - rp[:-k], axis=1) * 1e3)
        turned[k:] = np.maximum(turned[k:], np.degrees((rr[k:] * rr[:-k].inv()).magnitude()))
        if k == back:
            still = (moved < STILL_MM) & (turned < STILL_DEG)
            still[:back] = False
    hold_mask = (moved < STILL_MM) & (turned < STILL_DEG)
    hold_mask[:hold] = False
    e_p = np.linalg.norm(rp - mpp, axis=1) * 1e3
    e_r = np.degrees((rr * mrr.inv()).magnitude())

    out: dict[str, float] = {"hold_n": int(hold_mask.sum())}
    if hold_mask.any():
        out |= {"hold_p50_mm": float(np.percentile(e_p[hold_mask], 50)),
                "hold_p95_mm": float(np.percentile(e_p[hold_mask], 95)),
                "hold_p50_deg": float(np.percentile(e_r[hold_mask], 50)),
                "hold_p95_deg": float(np.percentile(e_r[hold_mask], 95))}

    # Moves: from the last still sample before the target leaves to the first after it returns.
    delays, residuals = [], []
    edges = np.flatnonzero(np.diff(still.astype(int)))
    starts = [k for k in edges if still[k] and not still[k + 1]]
    for a in starts:
        later = np.flatnonzero(still[a + 1:])
        if not len(later):
            break
        # `still` turns true STILL_S after the target stopped; that stop is where it ends.
        b = a + 1 + int(later[0]) - back
        if b <= a:
            continue
        d = rp[b] - rp[a]
        dist = float(np.linalg.norm(d)) * 1e3
        if not SMALL_MM[0] <= dist <= SMALL_MM[1]:
            continue
        u = d / np.linalg.norm(d)
        stop = min(len(g) - 1, b + int(1.0 * FINE_HZ))
        ref_along = (rp[a:stop] - rp[a]) @ u * 1e3
        arm_along = (mpp[a:stop] - mpp[a]) @ u * 1e3
        half = dist / 2
        t_ref = np.flatnonzero(ref_along >= half)
        t_arm = np.flatnonzero(arm_along >= half)
        if len(t_ref) and len(t_arm):
            delays.append((t_arm[0] - t_ref[0]) / FINE_HZ * 1e3)
        settle = min(len(g) - 1, b + int(0.5 * FINE_HZ))
        residuals.append(float(e_p[settle]))
    out["small_moves"] = len(residuals)
    if delays:
        out |= {"small_delay_p50_ms": float(np.median(delays)),
                "small_delay_p90_ms": float(np.percentile(delays, 90))}
    if residuals:
        out["small_settle_p50_mm"] = float(np.median(residuals))
    return out


def fine_from_recording(args: argparse.Namespace) -> int:
    """Score a `record` file (the operator's own teleop) for small motions. No ROS."""
    path = Path(args.recording).expanduser()
    tt, tp, tr = load_series(path, "target")
    mt, mp, mr = load_series(path, "pose")
    keep = (mt >= tt[0]) & (mt <= tt[-1])
    mt, mp, mr = mt[keep], mp[keep], mr[keep]
    # The target the controller held at each measured sample: the last one published.
    i = np.clip(np.searchsorted(tt, mt, side="right") - 1, 0, len(tt) - 1)
    out = fine(mt - mt[0], tp[i], tr[i], mp, mr)
    print(f"{path.name}: {mt[-1] - mt[0]:.0f} s")
    print("  " + "  ".join(f"{k} {v:.2f}" if isinstance(v, float) else f"{k} {v}"
                           for k, v in out.items()))
    return 0


def replay(args: argparse.Namespace) -> int:
    rec = Path(args.recording).expanduser()
    t, P, R = load_series(rec, "target")
    t = t - t[0]
    if args.seconds:
        keep = t <= args.seconds
        t, P, R = t[keep], P[keep], R[keep]
    trials = [(Path(g), *load_gains(g)) for g in args.gains]
    restore, _ = load_gains(args.restore)
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)

    cell = Cell(publish=True)
    time.sleep(0.5)
    active = cell.active_controllers()
    if CARTESIAN not in active:
        raise SystemExit(f"{CARTESIAN} is not active: arm the cell first (kuka_cell.py arm)")
    if cell.count_publishers(TARGET_TOPIC) > 1:
        raise SystemExit("another node publishes on /lbr/target_pose; close it first")
    vlim = cell.velocity_limits()
    print(f"recording {rec.name}: {len(t)} targets over {t[-1]:.1f} s; "
          f"{len(trials)} gains files; restore {args.restore}", flush=True)
    # The operator starts it, at the arm: a replay launched from a script or another terminal
    # against a cell someone left armed would move the arm with nobody holding the switch.
    if not sys.stdin.isatty():
        raise SystemExit("replay moves the arm and needs an operator at a terminal to start it")
    if input("THE ARM WILL MOVE. Enabling switch held, E-stop in reach? type 'go': ") != "go":
        raise SystemExit("not started")

    results: list[dict[str, Any]] = []
    try:
        for path, gains, options in trials:
            log: list[dict[str, Any]] = []
            cell.set_gains(gains)
            now = cell.get_gains(["task.k_pos_x", "task.d_pos_x", "task.k_rot_x", "task.d_rot_x"])
            lead = None
            if options.get("lead"):
                lead = (now["task.d_pos_x"] / now["task.k_pos_x"],
                        now["task.d_rot_x"] / now["task.k_rot_x"])
            print(f"\n== {path.name}: {now} lead={lead}", flush=True)
            time.sleep(0.3)
            aborted = None
            try:
                _approach(cell, args, vlim, P[0], R[0])
                _run(cell, args, vlim, t, P, R, lead, options.get("max_error_m"), log)
            except Abort as why:
                # Hold where the arm is, keep what was logged, and go on to the next file --
                # unless the cell itself is gone (stale pose, a second publisher).
                _brake(cell, restore, args.settle_s)
                at = f"{log[-1]['t']:.2f} s into the run" if log else "during the approach"
                aborted = f"{why} ({at})"
                print(f"   ABORTED: {aborted}; holding the measured pose", file=sys.stderr)
                if "stale" in str(why) or "another node" in str(why):
                    results.append({"gains_file": str(path), "aborted": aborted})
                    raise
            finally:
                (out / f"{path.stem}.jsonl").write_text("\n".join(json.dumps(r) for r in log))
            s = {"gains_file": str(path), "gains": now, "lead": lead,
                 "max_error_m": options.get("max_error_m"), "aborted": aborted}
            if len(log) > 50:
                s |= score(log, vlim)
            results.append(s)
            print("   " + "  ".join(f"{k} {v:.1f}" if isinstance(v, float) else f"{k} {v}"
                                     for k, v in s.items() if k not in ("gains", "gains_file",
                                                                         "lead", "max_error_m",
                                                                         "aborted")),
                  flush=True)
    except Abort as why:
        print(f"\nSTOPPED: {why}; the sweep cannot go on.", file=sys.stderr)
    except KeyboardInterrupt:
        _brake(cell, restore, 1.0)
        print("\nSTOPPED by ^C; holding the measured pose.", file=sys.stderr)
    finally:
        cell.set_gains(restore)
        print(f"gains restored from {args.restore}", flush=True)
        (out / "summary.json").write_text(json.dumps(
            {"recording": str(rec), "seconds": float(t[-1]), "trials": results}, indent=2))
        print(f"wrote {out / 'summary.json'}")
    return 1 if any(r.get("aborted") for r in results) else 0


def _brake(cell: Cell, restore: dict[str, float], seconds: float) -> None:
    """Restore the safe gains FIRST, then follow the measured pose for ``seconds``.

    26 Sep 2026, sweep 3: k 3000 / d 40 surged (A4 88 %), the run aborted, and the old hold --
    one measured pose published, then 2 s idle under the candidate gains -- left an underdamped
    arm swinging about a point it had already passed. FRI's CommandGuard dropped as the gains
    were finally restored, and the operator hit the E-stop. A target that follows the measured
    pose puts no spring on the arm at all; only damping acts, under the restored gains.
    """
    try:
        cell.set_gains(restore)
    finally:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            _hold(cell)
            time.sleep(1.0 / RATE_HZ)


def _hold(cell: Cell) -> None:
    """Target the measured pose: no spring, only damping -- the arm stops where it is."""
    with cell.lock:
        pose = cell.pose
    if pose is not None:
        cell.publish(pose[1], pose[2])


def set_gains(args: argparse.Namespace) -> int:
    """Put one gains file on the running controller, e.g. to teleoperate under a sweep's pick.

    A relaunch of the driver returns to controllers.yaml + controllers_umi.yaml.
    """
    gains, _ = load_gains(Path(args.gains))
    cell = Cell(publish=False)
    cell.set_gains(gains)
    names = ["task.k_pos_x", "task.d_pos_x", "task.k_rot_x", "task.d_rot_x", "nullspace.stiffness"]
    print(f"{CARTESIAN} now: {cell.get_gains(names)}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="cmd", required=True)
    rec = sub.add_parser("record", help="capture target/current pose and joints until ^C")
    rec.add_argument("out")
    st = sub.add_parser("set", help="set one gains file on the live controller (moves nothing)")
    st.add_argument("gains")
    stp = sub.add_parser("steps", help="write small steps about the current pose (moves nothing)")
    stp.add_argument("out")
    stp.add_argument("--mm", type=float, nargs="+", default=[1.0, 3.0])
    stp.add_argument("--deg", type=float, nargs="+", default=[0.5, 2.0])
    stp.add_argument("--hold-s", type=float, default=1.5)
    fn = sub.add_parser("fine", help="score a recording's small motions (no ROS, moves nothing)")
    fn.add_argument("recording")
    rep = sub.add_parser("replay", help="MOVES THE ARM: replay a recording under each gains file")
    rep.add_argument("recording")
    rep.add_argument("--gains", nargs="+", required=True)
    rep.add_argument("--restore", required=True, help="gains set on every way out")
    rep.add_argument("--out", required=True)
    rep.add_argument("--seconds", type=float, default=None, help="replay only the first N s")
    rep.add_argument("--settle-s", type=float, default=2.0)
    rep.add_argument("--floor-z", type=float, default=0.20)
    rep.add_argument("--abort-m", type=float, default=0.10)
    rep.add_argument("--abort-deg", type=float, default=30.0)
    rep.add_argument("--abort-speed", type=float, default=0.8,
                     help="fraction of a joint's velocity limit that aborts a run")
    args = parser.parse_args()
    if args.cmd == "fine":
        return fine_from_recording(args)
    # No rclpy signal handlers: they shut the context down on ^C, and the gains could then not
    # be restored. ^C is a KeyboardInterrupt here, and the finally blocks run with ROS alive.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    try:
        if args.cmd == "set":
            return set_gains(args)
        if args.cmd == "steps":
            return steps(args)
        return record(args) if args.cmd == "record" else replay(args)
    finally:
        rclpy.try_shutdown()
        for cell in _CELLS:
            cell.spinner.join(timeout=2.0)


if __name__ == "__main__":
    sys.exit(main())
