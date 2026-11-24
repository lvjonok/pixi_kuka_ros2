"""Replay a recorded target trajectory under each of several gains files and score the tracking.

    python scripts/track_tune.py replay REC.jsonl --gains A.yaml B.yaml --restore A.yaml --out DIR

REC.jsonl holds one JSON object per line; the rows with "k": "target" are used: "t" in seconds,
"p" the position in lbr_link_0, "q" the orientation as x, y, z, w. A gains file is a
`ros2 param load` file of doubles for /lbr/cartesian_impedance_controller. Beside ros__parameters
it may hold `replay: {lead: true}`, which sends each target (D/K) v ahead of itself, and
`replay: {max_error_m: 0.04}`, which holds the target within that distance of the arm.

DIR gets one <gains file>.jsonl per run, a row per 10 ms tick with the target sent and the pose
measured, and summary.json with the scores: the lag that best aligns the arm with the target and
the error left after it, the raw error, the peak joint speed as a fraction of its limit, the hold
error and the answer to small moves.

replay MOVES THE ARM, in T1 with the enabling switch held. It refuses unless
cartesian_impedance_controller is active, nothing else publishes /lbr/target_pose and an operator
types 'go' at a terminal. Before each run it walks the target from the measured pose to the
recording's first pose at 5 cm/s and 15 deg/s. It ends a run, restores the --restore gains and
follows the measured pose on a tracking error above --abort-m or --abort-deg, a joint above
--abort-speed of its velocity limit or a target under --floor-z, and goes on to the next file.
A stale pose or a second publisher ends the sweep. --restore is set on every way out.
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import rclpy.signals
import yaml
from geometry_msgs.msg import PoseStamped
from lbr_cell import (
    BASE_FRAME,
    CARTESIAN,
    JOINTS,
    NS,
    call,
    controller_states,
    joint_limits,
    spinning,
)
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import JointState

TARGET_TOPIC = f"{NS}/target_pose"
RATE_HZ = 100.0


def load_targets(path: Path) -> tuple[np.ndarray, np.ndarray, Rotation]:
    rows = [r for r in map(json.loads, path.read_text().splitlines()) if r["k"] == "target"]
    if len(rows) < 2:
        raise SystemExit(f"{path}: fewer than two target rows")
    t = np.array([r["t"] for r in rows])
    return t - t[0], np.array([r["p"] for r in rows]), Rotation.from_quat([r["q"] for r in rows])


def load_gains(path: str) -> tuple[dict[str, float], dict[str, Any]]:
    """``(parameters, replay options)`` of a gains file."""

    def flatten(tree: dict[str, Any], prefix: str = "") -> dict[str, float]:
        out: dict[str, float] = {}
        for key, value in tree.items():
            if isinstance(value, dict):
                out |= flatten(value, f"{prefix}{key}.")
            else:
                out[f"{prefix}{key}"] = float(value)
        return out

    (node,) = yaml.safe_load(Path(path).expanduser().read_text()).values()
    return flatten(node["ros__parameters"]), node.get("replay", {}) or {}


class Cell(Node):
    """The measured pose and joint speeds of the running cell, and its target topic."""

    def __init__(self) -> None:
        super().__init__("track_tune")
        self.lock = threading.Lock()
        self.pose: tuple[float, np.ndarray, Rotation] | None = None
        self.dq: np.ndarray | None = None
        self.q_hist: list[tuple[float, np.ndarray]] = []
        self.create_subscription(
            PoseStamped, f"{NS}/current_pose", self._on_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, f"{NS}/joint_states", self._on_joints, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(PoseStamped, TARGET_TOPIC, 1)

    def _on_pose(self, msg: PoseStamped) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        pose = (
            time.monotonic(),
            np.array([p.x, p.y, p.z]),
            Rotation.from_quat([o.x, o.y, o.z, o.w]),
        )
        with self.lock:
            self.pose = pose

    def _on_joints(self, msg: JointState) -> None:
        pos, vel = dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity))
        if not all(n in pos for n in JOINTS):
            return
        now = time.monotonic()
        q = np.array([pos[n] for n in JOINTS])
        reported = np.abs([vel.get(n, 0.0) for n in JOINTS])
        # lbr_fri_ros2's command guard differentiates the measured position over one FRI sample,
        # which reads higher than this velocity: the larger of it and a difference over about
        # 10 ms is kept.
        with self.lock:
            self.q_hist = [*self.q_hist[-5:], (now, q)]
            t_old, q_old = self.q_hist[0]
            fd = np.abs(q - q_old) / (now - t_old) if now - t_old > 1e-3 else np.zeros(7)
            self.dq = np.maximum(reported, fd)

    def measured(self) -> tuple[tuple[float, np.ndarray, Rotation] | None, np.ndarray | None]:
        with self.lock:
            return self.pose, self.dq

    def get_gains(self, names: list[str]) -> dict[str, float]:
        name = f"{NS}/{CARTESIAN}/get_parameters"
        result = call(self, GetParameters, name, GetParameters.Request(names=names))
        return {n: v.double_value for n, v in zip(names, result.values)}

    def set_gains(self, gains: dict[str, float]) -> None:
        params = [
            Parameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)),
            )
            for name, v in gains.items()
        ]
        name = f"{NS}/{CARTESIAN}/set_parameters"
        result = call(self, SetParameters, name, SetParameters.Request(parameters=params))
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


class Abort(Exception):
    """A run must end; the arm is left following its measured pose."""


def _guard(
    cell: Cell, args: argparse.Namespace, vlim: np.ndarray, p: np.ndarray, r: Rotation
) -> tuple[np.ndarray, Rotation, np.ndarray]:
    pose, dq = cell.measured()
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
        raise Abort(f"{JOINTS[j]} at {abs(dq[j]) / vlim[j]:.0%} of its velocity limit")
    return mp, mr, dq


def _approach(
    cell: Cell, args: argparse.Namespace, vlim: np.ndarray, p0: np.ndarray, r0: Rotation
) -> None:
    """Walk the target from the measured pose to (p0, r0), then settle."""
    _, sp, sr = cell.measured()[0]
    dist = float(np.linalg.norm(p0 - sp))
    ang = float((r0 * sr.inv()).magnitude())
    duration = max(dist / 0.05, np.degrees(ang) / 15.0, 0.5)
    slerp = Slerp([0.0, 1.0], Rotation.concatenate([sr, r0]))
    print(
        f"   approach: {dist * 1e3:.0f} mm, {np.degrees(ang):.1f} deg over {duration:.1f} s, "
        f"then settle {args.settle_s:.1f} s",
        flush=True,
    )
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


def _run(
    cell: Cell,
    args: argparse.Namespace,
    vlim: np.ndarray,
    t: np.ndarray,
    P: np.ndarray,
    R: Rotation,
    lead: tuple[float, float] | None,
    max_error_m: float | None,
    log: list[dict[str, Any]],
) -> None:
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
            _, mp0, _ = cell.measured()[0]
            gap = p - mp0
            if np.linalg.norm(gap) > max_error_m:
                p = mp0 + gap * (max_error_m / np.linalg.norm(gap))
        mp, mr, dq = _guard(cell, args, vlim, p, r)
        cell.publish(p, r)
        log.append(
            {
                "t": el,
                "ref_p": P[i].tolist(),
                "ref_q": R[i].as_quat().tolist(),
                "p": mp.tolist(),
                "q": mr.as_quat().tolist(),
                "dq": None if dq is None else dq.tolist(),
            }
        )
        if int(el) != shown:
            shown = int(el)
            print(
                f"   {el:5.1f}/{t[-1]:.0f} s  target {np.round((P[i] - P[0]) * 1e3, 1)} mm "
                f"{np.degrees((R[i] * R[0].inv()).magnitude()):.1f} deg  arm off by "
                f"{np.linalg.norm(P[i] - mp) * 1e3:.1f} mm "
                f"{np.degrees((R[i] * mr.inv()).magnitude()):.2f} deg",
                flush=True,
            )
        time.sleep(1.0 / RATE_HZ)


def _brake(cell: Cell, restore: dict[str, float], seconds: float) -> None:
    """Restore the gains first, then follow the measured pose for ``seconds``.

    A target on the measured pose puts no spring on the arm, only damping. Under gains that made
    it surge, a fixed target would leave the arm swinging about a point it has passed.
    """
    try:
        cell.set_gains(restore)
    finally:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            pose, _ = cell.measured()
            if pose is not None:
                cell.publish(pose[1], pose[2])
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

    # The fastest 100 ms. An arm faster than its target is catching up on a lag.
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
        "peak_joint": JOINTS[int(np.nanargmax(speed))],
    } | fine(t, ref_p, ref_r, mp, mr)


FINE_HZ = 100.0
STILL_S = 0.3
STILL_MM = 0.2
STILL_DEG = 0.05
HOLD_S = 1.0
SMALL_MM = (0.5, 10.0)


def fine(
    t: np.ndarray, ref_p: np.ndarray, ref_r: Rotation, mp: np.ndarray, mr: Rotation
) -> dict[str, float]:
    """How the arm answers small motions, which the lag score averages away.

    Hold error: where the arm sits once the target has been still for HOLD_S (p50 and p95, mm and
    deg). Small moves: target moves of SMALL_MM between two stills; how much later than the target
    the arm covers half of one, and the error 0.5 s after the target stops.
    """
    g = np.arange(t[0], t[-1], 1.0 / FINE_HZ)
    i = np.clip(np.searchsorted(t, g, side="right") - 1, 0, len(t) - 1)
    rp, mpp, rr, mrr = ref_p[i], mp[i], ref_r[i], mr[i]
    back = int(STILL_S * FINE_HZ)
    hold = int(HOLD_S * FINE_HZ)
    # The target's largest excursion over the last STILL_S (a move has ended) and HOLD_S (the arm
    # has had time to arrive).
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
        out |= {
            "hold_p50_mm": float(np.percentile(e_p[hold_mask], 50)),
            "hold_p95_mm": float(np.percentile(e_p[hold_mask], 95)),
            "hold_p50_deg": float(np.percentile(e_r[hold_mask], 50)),
            "hold_p95_deg": float(np.percentile(e_r[hold_mask], 95)),
        }

    delays, residuals = [], []
    edges = np.flatnonzero(np.diff(still.astype(int)))
    starts = [k for k in edges if still[k] and not still[k + 1]]
    for a in starts:
        later = np.flatnonzero(still[a + 1 :])
        if not len(later):
            break
        # `still` turns true STILL_S after the target stopped.
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
        out |= {
            "small_delay_p50_ms": float(np.median(delays)),
            "small_delay_p90_ms": float(np.percentile(delays, 90)),
        }
    if residuals:
        out["small_settle_p50_mm"] = float(np.median(residuals))
    return out


def replay(cell: Cell, args: argparse.Namespace) -> int:
    rec = Path(args.recording).expanduser()
    t, P, R = load_targets(rec)
    if args.seconds:
        keep = t <= args.seconds
        t, P, R = t[keep], P[keep], R[keep]
    trials = [(Path(g), *load_gains(g)) for g in args.gains]
    restore, _ = load_gains(args.restore)
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)

    time.sleep(0.5)
    if controller_states(cell).get(CARTESIAN) != "active":
        raise SystemExit(f"{CARTESIAN} is not active: arm the cell first")
    if cell.count_publishers(TARGET_TOPIC) > 1:
        raise SystemExit("another node publishes on /lbr/target_pose; close it first")
    vlim = joint_limits(cell)[2]
    print(
        f"recording {rec.name}: {len(t)} targets over {t[-1]:.1f} s; "
        f"{len(trials)} gains files; restore {args.restore}",
        flush=True,
    )
    # Started from a script against a cell left armed, it would move the arm with nobody on the
    # enabling switch.
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
                lead = (
                    now["task.d_pos_x"] / now["task.k_pos_x"],
                    now["task.d_rot_x"] / now["task.k_rot_x"],
                )
            print(f"\n== {path.name}: {now} lead={lead}", flush=True)
            time.sleep(0.3)
            aborted = None
            try:
                _approach(cell, args, vlim, P[0], R[0])
                _run(cell, args, vlim, t, P, R, lead, options.get("max_error_m"), log)
            except Abort as why:
                _brake(cell, restore, args.settle_s)
                at = f"{log[-1]['t']:.2f} s into the run" if log else "during the approach"
                aborted = f"{why} ({at})"
                print(f"   ABORTED: {aborted}; following the measured pose", file=sys.stderr)
                if "stale" in str(why) or "another node" in str(why):
                    results.append({"gains_file": str(path), "aborted": aborted})
                    raise
            finally:
                (out / f"{path.stem}.jsonl").write_text("\n".join(json.dumps(r) for r in log))
            s = {
                "gains_file": str(path),
                "gains": now,
                "lead": lead,
                "max_error_m": options.get("max_error_m"),
                "aborted": aborted,
            }
            if len(log) > 50:
                s |= score(log, vlim)
            results.append(s)
            hidden = ("gains", "gains_file", "lead", "max_error_m", "aborted")
            print(
                "   "
                + "  ".join(
                    f"{k} {v:.1f}" if isinstance(v, float) else f"{k} {v}"
                    for k, v in s.items()
                    if k not in hidden
                ),
                flush=True,
            )
    except Abort as why:
        print(f"\nSTOPPED: {why}; the sweep cannot go on.", file=sys.stderr)
    except KeyboardInterrupt:
        _brake(cell, restore, 1.0)
        print("\nSTOPPED by ^C; following the measured pose.", file=sys.stderr)
    finally:
        cell.set_gains(restore)
        print(f"gains restored from {args.restore}", flush=True)
        (out / "summary.json").write_text(
            json.dumps(
                {"recording": str(rec), "seconds": float(t[-1]), "trials": results}, indent=2
            )
        )
        print(f"wrote {out / 'summary.json'}")
    return 1 if any(r.get("aborted") for r in results) else 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    commands = parser.add_subparsers(dest="command", required=True)
    rep = commands.add_parser(
        "replay", help="MOVES THE ARM: replay a recording under each gains file"
    )
    rep.add_argument("recording")
    rep.add_argument("--gains", nargs="+", required=True)
    rep.add_argument("--restore", required=True, help="gains set on every way out")
    rep.add_argument("--out", required=True)
    rep.add_argument("--seconds", type=float, default=None, help="replay only the first N s")
    rep.add_argument("--settle-s", type=float, default=2.0)
    rep.add_argument("--floor-z", type=float, default=0.20)
    rep.add_argument("--abort-m", type=float, default=0.10)
    rep.add_argument("--abort-deg", type=float, default=30.0)
    rep.add_argument(
        "--abort-speed",
        type=float,
        default=0.8,
        help="fraction of a joint's velocity limit that ends a run",
    )
    args = parser.parse_args()
    # rclpy's signal handlers shut the context down on ^C, and the gains could then not be
    # restored: without them ^C is a KeyboardInterrupt and replay's finally runs with ROS alive.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    try:
        cell = Cell()
        with spinning(cell):
            return replay(cell, args)
    except RuntimeError as why:
        print(why, file=sys.stderr)
        return 1
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
