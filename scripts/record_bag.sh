#!/usr/bin/env bash
# Record everything needed to reconstruct what the arm did, for post-mortem rather than training.
#
# The parquet recorder (`iiwa-next record`) writes a curated schema at 100 Hz and is what the
# model trains on. This is the other thing: raw topics at full rate, plus /rosout, so that when
# something goes wrong the answer is in a file instead of in a terminal scrollback that has
# already wrapped.
#
# That is not hypothetical. An autonomous run tripped `lbr_fri_ros2::CommandGuard: Position not
# in limits`, and reconstructing why meant reading four tmux panes, none of which had the
# controller's internal state at the moment it happened. /rosout alone would have named the
# joint.
#
# Usage:
#   scripts/record_bag.sh ~/data/bags/excite_01
#
# Stop with Ctrl-C. The bag is written incrementally, so a bag from a run that crashed is still
# readable up to the crash -- which is the case that matters.

set -o pipefail

OUT="${1:?usage: record_bag.sh <output-dir> [extra ros2 bag args...]}"
shift

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS" || exit 1

# Match the environment the stack runs in. No `set -u`: ROS's setup scripts read unset variables.
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
source "$WS/.pixi/envs/jazzy/setup.sh" 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null

# What to record, and why each one earns its place.
TOPICS=(
    # The FRI state: measured and commanded joint position, measured torque, external torque.
    # This is the ground truth for everything -- what the arm did, at the rate it did it.
    /lbr/lbr_state
    /lbr/joint_states

    # What the arm was ASKED for, both halves of it. The difference between target_joint and the
    # q_ref that comes back through introspection is how you tell a nullspace that was commanded
    # from one that was ignored.
    /lbr/target_pose
    /lbr/target_joint
    /lbr/current_pose

    # The controller's own internals: every term of the control law, including q_ref, the task
    # error, and the individual torque contributions. 126 channels at 500 Hz, and the only place
    # some of them are observable at all.
    /lbr/controller_manager/introspection_data/names
    /lbr/controller_manager/introspection_data/values

    # The bridge's view: engaged state and the four clamp counters. A command channel sitting at
    # its clamp is a saturated distribution, and that fact is only visible here.
    /haply_teleop/status

    # Every log line from every node, with timestamps that line up with the data above. This is
    # the topic that turns "it refused for some reason" into a sentence naming the joint.
    /rosout
)

mkdir -p "$(dirname "$OUT")"

echo "recording to $OUT"
echo "topics:"
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Ctrl-C to stop. Afterwards, check the controller_manager pane for 'Overrun detected' --"
echo "if recording is costing the 500 Hz loop its deadline, drop the introspection topics first."
echo

exec ros2 bag record \
    --output "$OUT" \
    --storage mcap \
    --max-cache-size 268435456 \
    "$@" \
    "${TOPICS[@]}"
