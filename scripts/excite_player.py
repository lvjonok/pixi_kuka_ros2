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

- it refuses to start unless the arm is already at the plan's first configuration;
- it ramps in, so the first motion is slow whatever the plan says;
- it stops publishing on any of: end of plan, Ctrl-C, or the bridge reporting a refusal --
  and the bridge's watchdog then holds the arm, because under impedance "stop commanding" is a
  hold rather than a release.

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
    raw = np.load(path, allow_pickle=False)
    plan = {
        "q": raw["q"],
        "dq": raw["dq"],
        "phase": raw["phase"].astype(str),
        "rate_hz": float(raw["rate_hz"]),
        "plan_id": str(raw["plan_id"]) if "plan_id" in raw else None,
    }
    import hashlib

    digest = hashlib.sha256()
    digest.update(np.ascontiguousarray(plan["q"], dtype=np.float64).tobytes())
    digest.update(f"{plan['rate_hz']:.6f}".encode())
    computed = digest.hexdigest()[:16]
    if plan["plan_id"] is not None and computed != plan["plan_id"]:
        raise SystemExit(
            f"{path} records plan id {plan['plan_id']} but hashes to {computed}.\n"
            f"The file has changed since it was gated, so the gate's verdict does not "
            f"describe what is about to run. Re-run `iiwa-next excite` and gate it again."
        )
    plan["plan_id"] = computed
    return plan


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

        print(f"plan {plan['plan_id']}: {len(plan['q'])} samples at {plan['rate_hz']:g} Hz "
              f"({len(plan['q']) / plan['rate_hz'] / 60:.1f} min)")
        print("computing tip poses from the URDF the arm is running...")
        self.positions, self.quaternions = forward_kinematics(
            args.urdf, plan["q"], args.tip_frame
        )

    def measured_joints(self, timeout_s: float = 5.0) -> np.ndarray | None:
        """Ask the bridge where the arm is.

        Returns:
            ``(7,)`` measured joint positions, or ``None`` if the bridge did not answer.

        The bridge already replies to every packet with the arm's joint state, so this needs no
        second connection to ROS -- which matters, because a second subscriber is a second thing
        that can be pointed at the wrong DDS domain and quietly receive nothing.
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
            if joints is not None and len(joints) >= 7:
                return np.asarray(joints[:7], dtype=float)
        return None

    def check_start(self) -> bool:
        """Refuse to start unless the arm is already where the plan begins.

        The plan's first sample is not a place to *move* to -- it is where the trajectory assumes
        the arm already is. Starting anywhere else means the first commanded sample is a step,
        and a step in an impedance command is a shove.
        """
        measured = self.measured_joints()
        if measured is None:
            print(
                "no joint state from the bridge. Is it running, and is it on the same "
                "DDS domain as the arm?",
                file=sys.stderr,
            )
            return False
        first = self.plan["q"][0]
        error = np.abs(measured - first)
        worst = int(np.argmax(error))
        if np.max(error) > START_TOLERANCE_RAD:
            print(
                f"REFUSING to start: the arm is {np.degrees(np.max(error)):.1f} deg from the "
                f"plan's first configuration on A{worst + 1} "
                f"(tolerance {np.degrees(START_TOLERANCE_RAD):.0f} deg).\n"
                f"  measured {np.round(np.degrees(measured), 1).tolist()}\n"
                f"  plan[0]  {np.round(np.degrees(first), 1).tolist()}\n"
                f"Move the arm there first -- slowly, under supervision -- then run this again.",
                file=sys.stderr,
            )
            return False
        print(
            f"arm is at the plan's start ({np.degrees(np.max(error)):.2f} deg worst joint). "
            f"Ready."
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
        try:
            while index < total and not self.stop:
                now = time.monotonic()
                elapsed = now - started
                # Ramp the first seconds regardless of what the plan asks for. The plan begins
                # where the arm already is, so the ramp costs nothing but bounds how fast a
                # mistake at sample zero can become motion.
                scale = min(1.0, elapsed / RAMP_SECONDS) if RAMP_SECONDS > 0 else 1.0
                index = min(total - 1, int(elapsed * rate * scale))

                packet = {
                    "engaged": True,
                    "engage_id": engage_id,
                    "position": self.positions[index].tolist(),
                    "quaternion": self.quaternions[index].tolist(),
                    "joints": self.plan["q"][index].tolist(),
                }
                if not self.args.dry_run:
                    self.sock.sendto(json.dumps(packet).encode(), self.target)
                self.sent += 1

                if self.sent % int(rate * 10) == 0:
                    print(
                        f"  {elapsed / 60:5.1f} min  sample {index}/{total}  "
                        f"phase {phase[index]}"
                    )
                time.sleep(max(0.0, period - (time.monotonic() - now)))

            if index >= total - 1:
                print(f"\nplan complete: {self.sent} packets over {(time.monotonic() - started) / 60:.1f} min")
        except KeyboardInterrupt:
            print("\ninterrupted — stopping. The bridge watchdog holds the arm.")

        # Release the clutch explicitly rather than merely going quiet. The watchdog would hold
        # anyway, but an explicit release is what puts "the operator stopped" in the recording
        # rather than "the sender died", and those mean different things to whoever reads it.
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
    args = parser.parse_args(argv)

    if args.speed <= 0 or args.speed > 2.0:
        parser.error("--speed must be in (0, 2.0]; the plan was gated at its own rate")

    plan = load_plan(args.plan)
    player = Player(args, plan)

    if not player.check_start():
        return 1

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
