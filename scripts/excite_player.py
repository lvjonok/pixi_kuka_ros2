#!/usr/bin/env python3
"""Play a gated excitation plan into the teleop bridge, as if it were a hand on the Haply.

The plan is generated and gated on the laptop by ``iiwa-next excite``; this drives it. It speaks
the bridge's own UDP packet format and nothing else, which is the whole design:

    plan (.npz)  ->  this player  ->  UDP  ->  haply_teleop_bridge  ->  /lbr/target_pose
                                                                        /lbr/target_joint

Everything that keeps the arm safe already lives in the bridge and has been run on this cell with
a person holding the clutch: the 100 ms watchdog, the slew clamps, the workspace box, the
anchor-refusal on engage, and the error clamp that bounds the impedance force (50 mm x 500 N/m =
25 N). None of it is reimplemented here, and that is deliberate -- a second implementation of a
safety envelope is a second thing to get wrong, and the one that only runs unattended is the one
nobody exercises.

What this adds is the part that is specific to running without a hand on the clutch:

- **the plan advances on its own clock, and that clock only runs while the arm is following.**
  This is the one that matters. The first version advanced on wall-clock time, open loop, and
  never asked whether the arm was keeping up. When it fell behind, the commanded pose ran away,
  the pose error saturated at the bridge's 50 mm clamp, and the task term then pulled at its
  maximum force continuously -- straight through the 3 Nm ``joint_limit_repulsion``, which was
  sized for a human operator who would have noticed the arm resisting and let go. FRI's own
  ``CommandGuard`` caught it and neutralised the command, which is the only reason this is a
  paragraph rather than a repair. Every gate in this project checked the *plan*; nothing checked
  the *execution*;
- it refuses to start unless the arm is already at the plan's first configuration, in both joint
  and Cartesian terms, against the bridge's own tolerances;
- it ramps in, so the first motion is slow whatever the plan says;
- it stops on any of: end of plan, Ctrl-C, a joint coming within 20 deg of its stop, or the
  arm falling too far behind for too long -- and then **pins the configuration it stopped in**
  before releasing. "Stop commanding is a hold" is true of the tip and false of the arm: the
  task term holds six degrees of freedom and nothing holds the seventh, so a player that simply
  went quiet left the arm sliding along its self-motion manifold for another 22 seconds, ending
  up closer to the joint limit than when the abort fired.

Usage, from the KUKA workspace:

    pixi run -e jazzy python scripts/excite_player.py PLAN.npz --dry-run
    pixi run -e jazzy python scripts/excite_player.py PLAN.npz --speed 0.25
"""

from __future__ import annotations

import argparse
import json
import pathlib
import signal
import socket
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

#: How close the arm must already be to the plan's first configuration before anything is sent.
#: The bridge performs the same check in Cartesian terms and refuses an engage that is off; this
#: is the joint-space version, and it exists because the plan's first sample commands a nullspace
#: anchor too, which the bridge's pose-based check cannot see.
START_TOLERANCE_RAD = np.radians(3.0)

#: Seconds of ramp before the plan runs at its planned rate.
RAMP_SECONDS = 3.0

#: Hard joint limits for the iiwa 14 R820. The player aborts before the arm reaches them,
#: rather than leaving it to FRI's CommandGuard -- which does stop the arm, but by
#: neutralising commands mid-cycle, and twice took the controller_manager's 500 Hz loop with
#: it (100 ms overrun, 51 missed cycles). Stopping earlier and on purpose is cheaper.
JOINT_LIMITS_RAD = np.radians(np.array([170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0]))


def load_plan(path: pathlib.Path) -> dict:
    """Read a plan and verify it is the one that was gated.

    Args:
        path: The ``.npz`` written by ``iiwa-next excite --out``.

    Returns:
        Arrays and metadata.

    Raises:
        SystemExit: If the file has been modified since it was planned.

    The hash check is not ceremony. The gate ran on specific numbers, and a plan edited afterwards
    carries a verdict that is no longer about its contents -- while still loading cleanly, still
    having the right shape, and still moving the arm.
    """
    # Through iiwa_next's own loader, never a second copy of the hash.
    #
    # This used to recompute the digest here: sha256 over q and rate_hz. That was correct when it
    # was written and silently stopped being correct when the planner started folding the declared
    # payload into the identity -- the same joint trajectory gated for a bare flange and gated for
    # a 120 mm gripper are two different promises and must not share an id. The two
    # implementations then disagreed, and the disagreement surfaced here as the player refusing a
    # freshly gated plan:
    #
    #     excite_08.npz records plan id 93e47a6afc570a97 but hashes to 1c3c1c1573eb1040.
    #     The file has changed since it was gated ...
    #
    # which is a true sentence about the wrong thing: nothing had edited the file. Two copies of a
    # check are two checks, and the one that is not maintained is the one that fires.
    #
    # `iiwa_next` is installed into this workspace's jazzy env, so the canonical loader is
    # importable here and its ValueError carries the same refusal this used to raise.
    from iiwa_next.excite.plan import Session

    try:
        session = Session.load(path)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    return {
        "q": session.q,
        "dq": session.dq,
        "phase": np.asarray(session.phase).astype(str),
        "rate_hz": float(session.rate_hz),
        "plan_id": session.plan_id,
    }


def forward_kinematics(urdf: pathlib.Path, q: np.ndarray, tip_frame: str) -> tuple:
    """Tip poses for every sample of the plan.

    Args:
        urdf: Robot description.
        q: ``(n, 7)`` configurations.
        tip_frame: Frame to report.

    Returns:
        ``(n, 3)`` positions and ``(n, 4)`` xyzw quaternions.

    Computed here rather than stored in the plan so that the poses commanded to the arm come from
    the same description the arm is running. A plan carrying its own cached poses would keep
    working after the URDF changed, and command the old geometry.
    """
    import pinocchio as pin

    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    frame = model.getFrameId(tip_frame)
    positions = np.empty((len(q), 3))
    quaternions = np.empty((len(q), 4))
    for i, qi in enumerate(q):
        pin.framesForwardKinematics(model, data, qi)
        placement = data.oMf[frame]
        positions[i] = placement.translation
        quat = pin.Quaternion(placement.rotation)
        quaternions[i] = (quat.x, quat.y, quat.z, quat.w)
    return positions, quaternions


class Player:
    """Streams a plan to the bridge and listens to what it says back."""

    def __init__(self, args: argparse.Namespace, plan: dict) -> None:
        """Prepare the socket and precompute the commanded poses."""
        self.args = args
        self.plan = plan
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.target = (args.host, args.port)
        self.sent = 0
        self.stop = False
        self.measured_q: np.ndarray | None = None
        self.measured_p: np.ndarray | None = None
        self.last_commanded_p: np.ndarray | None = None
        self.frozen_ticks = 0
        self.view = None
        # Redraw at roughly 25 Hz whatever the streaming rate is. The browser gains nothing from
        # 200 updates a second and the send loop must not wait on rendering.
        self.view_every = max(1, int(round(plan["rate_hz"] * args.speed / 25.0)))

        print(f"plan {plan['plan_id']}: {len(plan['q'])} samples at {plan['rate_hz']:g} Hz "
              f"({len(plan['q']) / plan['rate_hz'] / 60:.1f} min)")
        print("computing tip poses from the URDF the arm is running...")
        self.positions, self.quaternions = forward_kinematics(
            args.urdf, plan["q"], args.tip_frame
        )

    def measured_state(self, timeout_s: float = 5.0) -> dict | None:
        """Ask the bridge where the arm is.

        Returns:
            The bridge's reply -- measured joints *and* the measured tip pose -- or ``None`` if it
            did not answer.

        The bridge already replies to every packet with both, so this needs no second connection
        to ROS -- which matters, because a second subscriber is a second thing that can be pointed
        at the wrong DDS domain and quietly receive nothing.

        Both halves are needed, and that is the whole point. The player used to check only the
        joints, against a 3-degree tolerance, while the bridge checks the *pose* against 5 mm.
        A configuration 0.4 degrees out passed here and was refused there -- 5.6 mm against a
        5 mm limit -- and the refusal arrived in a different pane, phrased as a teleoperation
        problem, with a number that grew every retry because this side kept advancing through
        the plan while the other side said no.
        """
        probe = json.dumps({"engaged": False, "engage_id": 0}).encode()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.sock.sendto(probe, self.target)
            time.sleep(0.05)
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                continue
            try:
                reply = json.loads(data)
            except json.JSONDecodeError:
                continue
            joints = reply.get("joints")
            if joints is not None and len(joints) >= 7 and reply.get("position"):
                return reply
        return None

    def _drain_replies(self) -> None:
        """Take the newest reply from the bridge and keep the arm's state from it."""
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(8192)
            except (BlockingIOError, OSError):
                break
            try:
                latest = json.loads(data)
            except json.JSONDecodeError:
                continue
        if latest is None:
            return
        joints = latest.get("joints")
        if joints is not None and len(joints) >= 7:
            self.measured_q = np.asarray(joints[:7], dtype=float)
        position = latest.get("position")
        if position is not None and len(position) == 3:
            self.measured_p = np.asarray(position, dtype=float)

    def pin_current(self, engage_id: int, seconds: float = 4.0) -> None:
        """Hold the arm's CONFIGURATION, not just its pose, before letting go.

        Args:
            engage_id: The engage this belongs to; reusing it avoids a fresh anchor check.
            seconds: How long to keep commanding the frozen configuration.

        "Stop commanding and the arm holds" is true of the tip and false of the arm. Under
        Cartesian impedance the six task degrees of freedom are held by the task term, and the
        seventh is held by nothing -- so when the player went quiet after aborting, the arm kept
        sliding along its self-motion manifold for another 22 seconds and ended up *closer* to
        the joint limit than when the abort fired: A2 went from 101.2 deg to 110.3, headroom
        18.8 deg to 9.7. The abort was correct and the release undid it.

        So instead of releasing, command the pose AND the joint configuration the arm is in right
        now. The nullspace target then pins the redundant degree of freedom where it stands
        rather than letting gravity choose, and the arm is actually stationary before the clutch
        is dropped.
        """
        self._drain_replies()
        if self.measured_q is None or self.measured_p is None:
            print("cannot pin: no joint state from the bridge", file=sys.stderr)
            return

        pose, quat = None, None
        try:
            import pinocchio as pin

            model = pin.buildModelFromUrdf(str(self.args.urdf))
            data = model.createData()
            frame = model.getFrameId(self.args.tip_frame)
            pin.framesForwardKinematics(model, data, self.measured_q)
            placement = data.oMf[frame]
            pose = placement.translation.copy()
            q_xyzw = pin.Quaternion(placement.rotation)
            quat = np.array([q_xyzw.x, q_xyzw.y, q_xyzw.z, q_xyzw.w])
        except Exception as exc:  # noqa: BLE001 - pinning must not fail on an import
            print(f"cannot pin: {exc}", file=sys.stderr)
            return

        held = self.measured_q.copy()
        near = self.joint_limit_headroom()
        print(
            f"pinning configuration for {seconds:.0f}s "
            f"(closest joint A{near[1] + 1} at {near[0]:.1f} deg headroom)"
            if near
            else f"pinning configuration for {seconds:.0f}s"
        )
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.sock.sendto(
                json.dumps(
                    {
                        "engaged": True,
                        "engage_id": engage_id,
                        "position": pose.tolist(),
                        "quaternion": quat.tolist(),
                        "joints": held.tolist(),
                    }
                ).encode(),
                self.target,
            )
            time.sleep(0.02)

        self._drain_replies()
        after = self.joint_limit_headroom()
        if after:
            print(f"  settled: closest joint A{after[1] + 1} at {after[0]:.1f} deg headroom")

    def joint_limit_headroom(self) -> tuple[float, int] | None:
        """How close the closest joint is to its stop, right now.

        Returns:
            ``(degrees_of_headroom, joint_index)``, or ``None`` before the first reply.

        The player watches the tip, because that is what the impedance law acts on. But the arm
        has been put on its stop twice, both times on A7, and neither time did the tip error say
        anything unusual -- a wrist roll barely moves the tip, so a joint can walk all the way
        into its limit while the Cartesian tracking looks healthy. The tip is simply not where
        this failure is visible, so it gets watched separately.
        """
        if self.measured_q is None:
            return None
        headroom = JOINT_LIMITS_RAD - np.abs(self.measured_q)
        worst = int(np.argmin(headroom))
        return float(np.degrees(headroom[worst])), worst

    def tracking_error_mm(self) -> float | None:
        """How far the arm's tip is from the pose most recently commanded.

        Returns:
            Millimetres, or ``None`` before the first reply from the bridge.

        Measured against the last *commanded* pose rather than against the plan's current sample,
        because that is the error the impedance law is acting on -- the one that becomes force.
        """
        if self.measured_p is None or self.last_commanded_p is None:
            return None
        return float(np.linalg.norm(self.last_commanded_p - self.measured_p)) * 1e3

    def start_view(self) -> None:
        """Bring up the live two-arm view, if one was asked for."""
        if not self.args.viser:
            return
        from excite_view import LiveView

        box = None
        if self.args.box_min and self.args.box_max:
            box = (np.asarray(self.args.box_min, float), np.asarray(self.args.box_max, float))
        self.view = LiveView(
            urdf=self.args.urdf,
            mesh_dirs=self.args.mesh_dir,
            plan_tips=self.positions,
            box=box,
            host=self.args.viser_host,
            port=self.args.viser,
        )

    def check_start(self) -> bool:
        """Refuse to start unless the arm is already where the plan begins.

        The plan's first sample is not a place to *move* to -- it is where the trajectory assumes
        the arm already is. Starting anywhere else means the first commanded sample is a step,
        and a step in an impedance command is a shove.

        Checked in **both** spaces, against the bridge's own tolerances, because the bridge is
        what actually decides. Checking only the joints let a 0.4-degree error through to be
        refused as 5.6 mm on the other side of a socket.
        """
        state = self.measured_state()
        if state is None:
            print(
                "no joint state from the bridge. Is it running, and is it on the same "
                "DDS domain as the arm?",
                file=sys.stderr,
            )
            return False

        measured = np.asarray(state["joints"][:7], dtype=float)
        first = self.plan["q"][0]
        error = np.abs(measured - first)
        worst = int(np.argmax(error))

        # The Cartesian comparison the bridge will make on the engage edge.
        measured_p = np.asarray(state["position"], dtype=float)
        gap_mm = float(np.linalg.norm(self.positions[0] - measured_p)) * 1e3
        ang_deg = 0.0
        if state.get("quaternion"):
            from scipy.spatial.transform import Rotation

            measured_r = Rotation.from_quat(np.asarray(state["quaternion"], dtype=float))
            ang_deg = float(
                np.degrees((Rotation.from_quat(self.quaternions[0]) * measured_r.inv()).magnitude())
            )

        joint_bad = float(np.max(error)) > START_TOLERANCE_RAD
        pose_bad = gap_mm > self.args.anchor_tol_mm or ang_deg > self.args.anchor_tol_deg
        if joint_bad or pose_bad:
            print(
                f"REFUSING to start.\n"
                f"  joints : worst A{worst + 1} off by {np.degrees(np.max(error)):.2f} deg "
                f"(tolerance {np.degrees(START_TOLERANCE_RAD):.0f} deg)"
                f"{'  <-- FAILS' if joint_bad else ''}\n"
                f"  pose   : {gap_mm:.1f} mm / {ang_deg:.2f} deg from measured "
                f"(the bridge refuses above {self.args.anchor_tol_mm:.0f} mm / "
                f"{self.args.anchor_tol_deg:.0f} deg)"
                f"{'  <-- FAILS' if pose_bad else ''}\n"
                f"  measured {np.round(np.degrees(measured), 2).tolist()}\n"
                f"  plan[0]  {np.round(np.degrees(first), 2).tolist()}\n\n"
                f"The arm rests a few millimetres off any nominal pose, because under impedance "
                f"it sags to where gravity and the spring balance. So plan from the joints the "
                f"arm is ACTUALLY at, not from the nominal ones:\n\n"
                f"  iiwa-next excite ... --start-deg "
                f"{' '.join(f'{v:.2f}' for v in np.degrees(measured))}\n",
                file=sys.stderr,
            )
            return False
        print(
            f"arm is at the plan's start: {np.degrees(np.max(error)):.2f} deg worst joint, "
            f"{gap_mm:.1f} mm / {ang_deg:.2f} deg in pose. Ready."
        )
        return True

    def run(self) -> int:
        """Stream the plan.

        Returns:
            Process exit code.
        """
        rate = self.plan["rate_hz"] * self.args.speed
        period = 1.0 / rate
        total = len(self.plan["q"])
        phase = self.plan["phase"]
        engage_id = int(time.time()) % 100000

        print(
            f"streaming at {rate:g} Hz (speed x{self.args.speed:g}); "
            f"Ctrl-C stops publishing and the bridge holds"
        )
        started = time.monotonic()
        index = 0
        # The plan has its own clock, and it only runs while the arm is keeping up. Wall-clock
        # time is NOT the plan's time.
        plan_time = 0.0
        last_tick = time.monotonic()
        behind_since: float | None = None
        self.frozen_ticks = 0
        try:
            while index < total and not self.stop:
                now = time.monotonic()
                dt, last_tick = now - last_tick, now
                elapsed = now - started
                # Ramp the first seconds regardless of what the plan asks for. The plan begins
                # where the arm already is, so the ramp costs nothing but bounds how fast a
                # mistake at sample zero can become motion.
                scale = min(1.0, elapsed / RAMP_SECONDS) if RAMP_SECONDS > 0 else 1.0

                # --- the interlock -------------------------------------------------------
                # Advance the plan only while the arm is following it. Without this the plan is
                # open loop in wall-clock time: if the arm falls behind, the command runs away,
                # the pose error saturates at the bridge's clamp, and the task term then pulls at
                # its maximum force continuously. That is not a hypothetical -- it drove a joint
                # into its limit, because a saturated task term walks straight through the 3 Nm
                # `joint_limit_repulsion` that was sized for a human who would have let go.
                near = self.joint_limit_headroom()
                if near is not None and near[0] < self.args.abort_headroom_deg:
                    print(
                        f"\nABORTING: A{near[1] + 1} is {near[0]:.1f} deg from its stop "
                        f"(floor {self.args.abort_headroom_deg:.0f} deg). The arm is walking into "
                        f"a joint limit -- this is the failure that ended two earlier runs, and "
                        f"the tip tracking looks fine while it happens.",
                        file=sys.stderr,
                    )
                    break

                behind_mm = self.tracking_error_mm()
                if behind_mm is not None and behind_mm > self.args.freeze_mm:
                    behind_since = behind_since or now
                    self.frozen_ticks += 1
                    if now - behind_since > self.args.abort_after_s:
                        print(
                            f"\nABORTING: the arm has been more than {self.args.freeze_mm:.0f} mm "
                            f"behind the plan for {self.args.abort_after_s:.0f}s "
                            f"(now {behind_mm:.0f} mm). It is not following, and continuing "
                            f"would keep the task term pinned at maximum force.",
                            file=sys.stderr,
                        )
                        break
                else:
                    behind_since = None
                    plan_time += dt * scale

                index = min(total - 1, int(plan_time * rate))

                packet = {
                    "engaged": True,
                    "engage_id": engage_id,
                    "position": self.positions[index].tolist(),
                    "quaternion": self.quaternions[index].tolist(),
                    "joints": self.plan["q"][index].tolist(),
                }
                if not self.args.dry_run:
                    self.sock.sendto(json.dumps(packet).encode(), self.target)
                self.last_commanded_p = self.positions[index]
                self.sent += 1

                # Drain the bridge's replies every tick. Needed even without the view: the bridge
                # answers every packet, and a receive buffer nobody reads fills up and starts
                # dropping -- silently, and on the socket this process depends on.
                self._drain_replies()
                if self.view is not None and self.sent % self.view_every == 0:
                    self.view.update(
                        q_plan=self.plan["q"][index],
                        q_measured=self.measured_q,
                        target_position=self.positions[index],
                        target_quaternion=self.quaternions[index],
                        measured_position=self.measured_p,
                        index=index,
                        total=total,
                        phase=str(phase[index]),
                    )

                if self.sent % int(rate * 10) == 0:
                    print(
                        f"  {elapsed / 60:5.1f} min  sample {index}/{total}  "
                        f"phase {phase[index]}"
                    )
                time.sleep(max(0.0, period - (time.monotonic() - now)))

            if index >= total - 1:
                wall = (time.monotonic() - started) / 60
                frozen = self.frozen_ticks / max(1, self.sent)
                print(
                    f"\nplan complete: {self.sent} packets over {wall:.1f} min "
                    f"(plan time {plan_time / 60:.1f} min)"
                )
                # A session that spent much of its time frozen is a session whose plan asked for
                # motion this arm cannot track, and the recorded data is correspondingly skewed
                # toward whatever the arm does while catching up. Worth knowing before training.
                if frozen > 0.01:
                    print(
                        f"  the arm was behind the command for {100 * frozen:.0f}% of ticks -- "
                        f"the plan is faster than this arm tracks; lower --speed"
                    )
        except KeyboardInterrupt:
            print("\ninterrupted — pinning the arm where it stands.")

        self.pin_current(engage_id)

        # Only now release. The watchdog would hold anyway, but an explicit release is what puts
        # "the operator stopped" in the recording rather than "the sender died", and those mean
        # different things to whoever reads it.
        for _ in range(10):
            self.sock.sendto(
                json.dumps({"engaged": False, "engage_id": engage_id}).encode(), self.target
            )
            time.sleep(0.02)
        return 0


def main(argv: list[str] | None = None) -> int:
    """Entry point."""
    parser = argparse.ArgumentParser(
        description="Play a gated excitation plan into the teleop bridge."
    )
    parser.add_argument("plan", type=pathlib.Path, help="the .npz from `iiwa-next excite --out`")
    parser.add_argument("--urdf", type=pathlib.Path, required=True)
    parser.add_argument("--tip-frame", default="lbr_link_ee")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9301)
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="playback rate multiplier. Run the first session well below 1.0.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="do everything except send; still checks the start configuration",
    )
    parser.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    # These must match the bridge's --anchor-tol-m / --anchor-tol-deg. They are repeated rather
    # than imported because the bridge is a separate process that may be running with its own
    # flags; if you change them there, change them here, and the player's message will still be
    # about the same numbers the bridge is enforcing.
    # The interlock. 25 mm is half the bridge's 50 mm error clamp, so the plan stops
    # advancing well before the task term saturates rather than after.
    parser.add_argument("--freeze-mm", type=float, default=25.0)
    # Well outside the 14 deg where joint_limit_repulsion now engages, so the player
    # stops before the controller is relying on a spring to save it.
    parser.add_argument("--abort-headroom-deg", type=float, default=20.0)
    parser.add_argument("--abort-after-s", type=float, default=5.0)
    parser.add_argument("--anchor-tol-mm", type=float, default=5.0)
    parser.add_argument("--anchor-tol-deg", type=float, default=3.0)
    parser.add_argument(
        "--viser",
        type=int,
        nargs="?",
        const=8095,
        default=None,
        metavar="PORT",
        help="serve a live view: the plan as a translucent ghost, the arm solid, and the tip "
        "target frame. This is where an elbow that is not following becomes visible -- the "
        "pose tracks either way, so no number in the run reports it.",
    )
    parser.add_argument("--viser-host", default="0.0.0.0")
    parser.add_argument(
        "--mesh-dir",
        type=pathlib.Path,
        action="append",
        default=None,
        help="root for package:// mesh references; repeatable. Needed for the view to draw "
        "anything but empty frames.",
    )
    parser.add_argument("--box-min", type=float, nargs=3, default=[0.35, -0.35, 0.25])
    parser.add_argument("--box-max", type=float, nargs=3, default=[0.80, 0.35, 0.75])
    args = parser.parse_args(argv)

    if args.speed <= 0 or args.speed > 2.0:
        parser.error("--speed must be in (0, 2.0]; the plan was gated at its own rate")

    plan = load_plan(args.plan)
    player = Player(args, plan)

    if not player.check_start():
        return 1

    # Before the prompt, not after. The view is meant to be looked at while deciding whether to
    # start, and it already shows the ghost at the plan's first sample sitting on top of the arm.
    player.start_view()

    if not args.yes and not args.dry_run:
        # The one place a person is asked. Everything after this runs with nobody's hand on the
        # clutch, which is a different risk class from anything this cell has done so far.
        print(
            f"\nAbout to drive the arm autonomously for "
            f"{len(plan['q']) / plan['rate_hz'] / 60 / args.speed:.1f} minutes.\n"
            f"Confirm the cell is clear and you are at the e-stop."
        )
        if input("type 'run' to start: ").strip() != "run":
            print("aborted")
            return 1

    def handle(_signum: int, _frame: object) -> None:
        player.stop = True

    signal.signal(signal.SIGINT, handle)
    signal.signal(signal.SIGTERM, handle)
    return player.run()


if __name__ == "__main__":
    raise SystemExit(main())
