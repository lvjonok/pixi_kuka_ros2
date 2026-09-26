"""Watch the joint speeds the way lbr_fri_ros2's CommandGuard does, and say which joint and why.

Read-only. The guard (src/lbr_fri_ros2_stack/lbr_fri_ros2/src/guards/command_guard.cpp,
`command_in_velocity_limits_`) runs in the driver on every FRI cycle while commanding:

    |q_measured - q_measured_previous| / sample_time  >  URDF velocity limit  ->  invalid

and on the first invalid cycle it overrides the command to neutral, stops updating its previous
position, and so fails every later cycle against that stale one -- until FRI leaves
COMMANDING_ACTIVE and the session is over. Its log then names whichever joint it checks FIRST
that has moved since (A1 almost always), not the joint that tripped. This names that joint.

Two speeds per joint, from /lbr/lbr_state (published by lbr_state_broadcaster every cycle):

* true  -- displacement over the robot's own timestamp difference;
* guard -- the same displacement over `sample_time`, which is what the guard divides by. The two
  differ when a sample is missing between two messages: a cycle the driver missed doubles the
  guard's number without the arm going any faster. Gaps are counted, so a trip at a moderate
  true speed with a gap beside it reads as timing, not motion. (A gap here can also be a message
  this subscriber dropped; the guard itself only sees the driver's.)

    pixi run -e jazzy guard-watch                    # live, one line per 0.5 s
    pixi run -e jazzy guard-watch --log ~/data/guard.jsonl --warn 0.7
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

from lbr_fri_idl.msg import LBRState

NS = "/lbr"
JOINT_NAMES = tuple(f"lbr_A{i + 1}" for i in range(7))
# KUKA::FRI::ESessionState
COMMANDING_ACTIVE = 4
SESSION_NAMES = {0: "IDLE", 1: "MONITORING_WAIT", 2: "MONITORING_READY", 3: "COMMANDING_WAIT",
                 4: "COMMANDING_ACTIVE"}


class GuardWatch(Node):
    """Per-sample joint speed against the limits the driver enforces."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("guard_watch")
        self.args = args
        self.vlim = self._velocity_limits()
        self.prev: tuple[float, np.ndarray] | None = None
        self.session: int | None = None
        self.window_true = np.zeros(7)
        self.window_guard = np.zeros(7)
        self.window_gaps = 0
        self.window_n = 0
        self.peak_true = np.zeros(7)
        self.peak_guard = np.zeros(7)
        self.log = Path(args.log).expanduser().open("a") if args.log else None
        print("limits [deg/s]: " + "  ".join(
            f"{n[-2:]} {math.degrees(v):.0f}" for n, v in zip(JOINT_NAMES, self.vlim)), flush=True)
        self.create_subscription(LBRState, f"{NS}/lbr_state", self._on_state,
                                 qos_profile_sensor_data)
        self.create_timer(args.period, self._report)

    def _velocity_limits(self) -> np.ndarray:
        """The URDF limits the guard is built from (system_interface.cpp)."""
        got: list[str] = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.create_subscription(String, f"{NS}/robot_description",
                                       lambda m: got.append(m.data), qos)
        deadline = time.monotonic() + 10.0
        while not got and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        if not got:
            raise SystemExit(f"no {NS}/robot_description within 10 s -- is the stack up?")
        model = pin.buildModelFromXML(got[0])
        return np.array(
            [model.velocityLimit[model.joints[model.getJointId(n)].idx_v] for n in JOINT_NAMES]
        )

    def _on_state(self, msg: LBRState) -> None:
        stamp = msg.time_stamp_sec + 1e-9 * msg.time_stamp_nano_sec
        q = np.asarray(msg.measured_joint_position, dtype=float)
        if msg.session_state != self.session:
            if self.session is not None:
                print(f"  session {SESSION_NAMES.get(self.session, self.session)} -> "
                      f"{SESSION_NAMES.get(msg.session_state, msg.session_state)}", flush=True)
                self._write({"event": "session", "from": self.session, "to": msg.session_state})
            self.session = msg.session_state
        if self.prev is not None and msg.sample_time > 0:
            t0, q0 = self.prev
            dt = stamp - t0
            if dt > 0:
                dq = np.abs(q - q0)
                true = dq / dt / self.vlim
                guard = dq / msg.sample_time / self.vlim
                gap = dt > 1.5 * msg.sample_time
                self.window_true = np.maximum(self.window_true, true)
                self.window_guard = np.maximum(self.window_guard, guard)
                self.window_gaps += int(gap)
                self.window_n += 1
                if guard.max() >= self.args.warn and msg.session_state == COMMANDING_ACTIVE:
                    j = int(guard.argmax())
                    print(f"  !! {JOINT_NAMES[j]} guard {guard[j]:.0%} true {true[j]:.0%}"
                          f"{'  (GAP ' + format(dt * 1e3, '.1f') + ' ms)' if gap else ''}",
                          flush=True)
                    self._write({"event": "excursion", "joint": JOINT_NAMES[j],
                                 "guard_frac": float(guard[j]), "true_frac": float(true[j]),
                                 "dt_ms": dt * 1e3, "sample_time_ms": msg.sample_time * 1e3})
        self.prev = (stamp, q)

    def _report(self) -> None:
        if not self.window_n:
            print("  no /lbr/lbr_state -- is lbr_state_broadcaster active?", flush=True)
            return
        self.peak_true = np.maximum(self.peak_true, self.window_true)
        self.peak_guard = np.maximum(self.peak_guard, self.window_guard)
        j = int(self.window_guard.argmax())
        bars = "  ".join(f"{n[-2:]} {v:4.0%}" for n, v in zip(JOINT_NAMES, self.window_true))
        print(f"{SESSION_NAMES.get(self.session, self.session)[:10]:10s} true {bars} | guard max "
              f"{JOINT_NAMES[j][-2:]} {self.window_guard[j]:.0%} | gaps {self.window_gaps}/"
              f"{self.window_n}", flush=True)
        self.window_true[:] = 0
        self.window_guard[:] = 0
        self.window_gaps = self.window_n = 0

    def _write(self, row: dict) -> None:
        if self.log:
            self.log.write(json.dumps({"t": time.time(), **row}) + "\n")
            self.log.flush()

    def summary(self) -> str:
        return "session peaks, true: " + "  ".join(
            f"{n[-2:]} {v:.0%}" for n, v in zip(JOINT_NAMES, self.peak_true)
        ) + " | guard: " + "  ".join(
            f"{n[-2:]} {v:.0%}" for n, v in zip(JOINT_NAMES, self.peak_guard))


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--warn", type=float, default=0.7,
                        help="print every sample whose guard speed is at least this fraction")
    parser.add_argument("--period", type=float, default=0.5, help="seconds per summary line")
    parser.add_argument("--log", default=None, help="append excursions and session changes here")
    args = parser.parse_args()
    rclpy.init()
    node = GuardWatch(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(node.summary())
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
