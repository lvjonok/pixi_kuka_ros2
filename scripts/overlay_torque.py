"""The torque each position_source leaves to crisp, and the torque it hands to Sunrise's spring.

docs/position_source.md gate 2: `ik_target` must answer small steps with less lag (track_tune's
summary.json) AND less overlay torque than `measured`. track_tune logs poses only; the torques
are in /lbr/lbr_state, so a replay runs beside a `ros2 bag record` of it and of the
passthrough's ~/state (the runbook's command), and this reads the bags back. Per joint, over
every sample in a bag:

* overlay  -- FRI's commanded_torque, the overlay crisp sent (Sunrise's spring is not in it);
* spring   -- (commanded - measured joint position) x sunrise stiffness: what Sunrise's spring
  pushed with. ~0 under measured, by construction;
* clamps   -- if the bag has /lbr/fri_position_passthrough_controller/state, the fraction of
  updates the gap clamp and the step limit cut the output, and the stale fraction.

    pixi run -e jazzy python scripts/overlay_torque.py $OUT/steps/{measured,lagged,ik_target}/bag \
        [--stiffness 200]                    # docs/position_source.md, "Cell test (gate 2)"

Reads files only: no ROS graph, moves nothing. A bag covers the whole replay (approach and
settle included), so compare bags of the same recording.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

LBR_STATE = "/lbr/lbr_state"
PASSTHROUGH_STATE = "/lbr/fri_position_passthrough_controller/state"
JOINTS = 7
STATE_SIZE = 2 + 5 * JOINTS  # [mode, stale, q_meas, q_ik, q_out, gap_clamped, step_clamped]


def summarize(
    commanded_torque: np.ndarray,
    commanded_q: np.ndarray,
    measured_q: np.ndarray,
    stiffness: np.ndarray,
    passthrough: np.ndarray | None = None,
) -> dict[str, np.ndarray | float]:
    """Per-joint RMS / p95 of the overlay and the spring torque; clamp fractions if given.

    The first three are (N, 7) from LBRState; `passthrough` is (M, 37) ~/state rows.
    """
    spring = (commanded_q - measured_q) * stiffness
    out: dict[str, np.ndarray | float] = {
        "samples": float(len(commanded_torque)),
        "overlay_rms": np.sqrt(np.mean(commanded_torque**2, axis=0)),
        "overlay_p95": np.percentile(np.abs(commanded_torque), 95, axis=0),
        "spring_rms": np.sqrt(np.mean(spring**2, axis=0)),
        "spring_p95": np.percentile(np.abs(spring), 95, axis=0),
    }
    if passthrough is not None and len(passthrough):
        out["modes"] = np.unique(passthrough[:, 0])
        out["stale_frac"] = float(np.mean(passthrough[:, 1] != 0))
        out["gap_clamped_frac"] = np.mean(passthrough[:, 23:30] != 0, axis=0)
        out["step_clamped_frac"] = np.mean(passthrough[:, 30:37] != 0, axis=0)
    return out


def read_bag(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray | None]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if LBR_STATE not in types:
        raise SystemExit(f"{path}: no {LBR_STATE} (record it beside the replay)")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[LBR_STATE, PASSTHROUGH_STATE]))
    classes = {name: get_message(types[name]) for name in types}
    tau, q_cmd, q_meas, rows = [], [], [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        if topic == LBR_STATE:
            tau.append(msg.commanded_torque)
            q_cmd.append(msg.commanded_joint_position)
            q_meas.append(msg.measured_joint_position)
        elif len(msg.data) == STATE_SIZE:
            rows.append(msg.data)
    if not tau:
        raise SystemExit(f"{path}: {LBR_STATE} is empty")
    return np.array(tau), np.array(q_cmd), np.array(q_meas), np.array(rows) if rows else None


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument(
        "--stiffness",
        type=float,
        nargs="+",
        default=[200.0],
        help="the running Sunrise app's joint stiffness, Nm/rad (one, or seven)",
    )
    args = parser.parse_args()
    stiffness = np.broadcast_to(np.asarray(args.stiffness, dtype=float), (JOINTS,))

    def row(label: str, v: np.ndarray, unit: str = "") -> str:
        return f"  {label:<18}" + " ".join(f"{x:5.1f}%" if unit else f"{x:6.2f}" for x in v)

    for path in args.bags:
        tau, q_cmd, q_meas, rows = read_bag(path.expanduser())
        s = summarize(tau, q_cmd, q_meas, stiffness, rows)
        print(f"{path}: {int(s['samples'])} samples")
        print(f"  {'':<18}" + " ".join(f"{'A' + str(j + 1):>6}" for j in range(JOINTS)))
        print(row("overlay rms Nm", s["overlay_rms"]))
        print(row("overlay p95 Nm", s["overlay_p95"]))
        print(row("spring rms Nm", s["spring_rms"]))
        print(row("spring p95 Nm", s["spring_p95"]))
        if "modes" in s:
            print(
                f"  modes {s['modes'].astype(int).tolist()} (0 measured, 1 lagged, "
                f"2 ik_target), stale {s['stale_frac']:.1%}"
            )
            print(row("gap clamped", s["gap_clamped_frac"] * 100, "%"))
            print(row("step clamped", s["step_clamped_frac"] * 100, "%"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
