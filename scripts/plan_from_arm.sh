#!/usr/bin/env bash
# Read where the arm actually is, and plan an excitation session starting from there.
#
# The two steps belong together. A plan begins at the configuration the arm is standing in, and
# the arm rests a few millimetres off any nominal pose because under impedance it settles where
# gravity and the spring balance -- so planning from HOME_DEGREES rather than from the measured
# joints produced a first sample 5.6 mm away, which the bridge refused against its 5 mm anchor
# tolerance. Reading and planning in one command removes the transcription step where that
# mistake lives.
#
# Usage:
#   scripts/plan_from_arm.sh ~/data/excite_04.npz [--scaffolds 12] [--seed 21]

set -o pipefail

OUT="${1:?usage: plan_from_arm.sh <output.npz> [extra iiwa-next excite args...]}"
shift

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NEXT="$HOME/github.com/lvjonok/iiwa_next"
URDF="$HOME/data/iiwa14.urdf"
MESHES="$WS/install/lbr_iiwa14_r820_description/share"
SRDF="$WS/src/lbr_fri_ros2_stack/lbr_moveit_config/iiwa14_moveit_config/config/iiwa14.srdf"

cd "$WS" || exit 1
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
source "$WS/.pixi/envs/jazzy/setup.sh" 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null

echo "reading the arm's joints..."
START=$("$WS/.pixi/envs/jazzy/bin/python" - <<'PY'
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

rclpy.init()
node = Node("plan_from_arm")
seen = []
node.create_subscription(
    JointState, "/lbr/joint_states", lambda m: seen.append(m), qos_profile_sensor_data
)
deadline = time.monotonic() + 10
while time.monotonic() < deadline and len(seen) < 60:
    rclpy.spin_once(node, timeout_sec=0.1)
if len(seen) < 20:
    sys.exit("no joint states — is the stack up and on ROS_DOMAIN_ID 42?")

# Average the tail rather than take one sample: the arm is holding under impedance, not frozen,
# and a single sample carries whatever it was doing at that instant.
block = np.array([list(m.position[:7]) for m in seen[-40:]])
q = np.degrees(block.mean(axis=0))
drift = np.degrees(block.std(axis=0)).max()
if drift > 0.05:
    print(f"# WARNING arm is still moving ({drift:.3f} deg spread); let it settle", file=sys.stderr)

limits = np.array([170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0])
head = limits - np.abs(q)
print("# joints deg  : " + " ".join(f"{v:+8.2f}" for v in q), file=sys.stderr)
print("# headroom    : " + " ".join(f"{v:8.1f}" for v in head), file=sys.stderr)
if head.min() < 25:
    print(
        f"# WARNING A{int(np.argmin(head)) + 1} has only {head.min():.1f} deg to its stop. "
        f"Home the arm before planning from here.",
        file=sys.stderr,
    )
print(" ".join(f"{v:.3f}" for v in q))
PY
) || exit 1

echo "starting from: $START"
echo

exec "$NEXT/.pixi/envs/dev/bin/python" -m iiwa_next.cli excite \
    --urdf "$URDF" --mesh-dir "$MESHES" --srdf "$SRDF" \
    --start-deg $START \
    --out "$OUT" "$@"
