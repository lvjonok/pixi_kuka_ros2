# What the FRI position field carries under the torque overlay

Branch `feat/drake-joint-impedance`. Status: implemented, gate 1 (mock) passed; amended after
review (see [Changes after review](#changes-after-review)).

## Why

In `TORQUE` client command mode FRI sends a joint position *and* a torque every cycle. Sunrise
runs its joint impedance controller around that position (`LBRServer.java`: 200 Nm/rad on every
joint; damping 0.3 by the operator's recollection, or KUKA's default 0.7 if the cabinet's
LBRServer has no `setDampingForAllJoints` call, see docs/sunrise_setup.md) and adds our torque on
top. Today
`fri_position_passthrough_controller` writes the **measured** position into that field, so:

- Sunrise's spring is centred on where the arm already is and holds nothing; all stiffness is
  crisp's, computed on this PC across the network and lbr_fri_ros2's filters.
- KUKA's internal friction compensation reportedly engages only on a position difference
  (drake-iiwa-driver `kuka_driver.cc`: "a hack to trigger the friction observer within the IIWA.
  The friction observer is only triggered when there is a position difference"; Explicit-FRI's
  README says the same). On this cell the joints hold up to ~6 Nm (A1, A2) before they move, and
  policy chunks are tracked with ~200 ms (position) / ~300 ms (rotation) lag.

Drake's driver fills the field with the commanded joint positions (its IK/planner target) under
joint impedance 1000/1000/1000/500/500/500/500 Nm/rad, damping 1.0. Its torque-only mode, with
Sunrise's spring at zero, sends an IIR-lagged copy of the measured position. This branch makes
the field selectable, keeping every name and topic the rest of the cell uses.

## Interface (what stays, what is new)

Unchanged: the controller name `fri_position_passthrough_controller`, its type
`pixi_kuka_ros2/FRIPositionPassthroughController`, the claimed interfaces (`<joint>/position`
command and state), the REST/ARMED switching in `scripts/kuka_crisp.py` (which adds one refusal,
under Safety notes), crisp and its topics and
gains files, `/lbr/state`, the launch files. With `position_source: measured` (the default) the
controller behaves exactly as before.

New parameters of `fri_position_passthrough_controller` (all read at `on_configure` and again at
`on_activate`; a change takes effect at the next activation, i.e. the next REST -> ARMED; the
active mode is logged at every activation; a value that fails validation fails the activation
with FAILURE, so the controller stays inactive and configured and arms again once it is fixed):

| parameter | type | default | meaning |
|---|---|---|---|
| `position_source` | string | `measured` | `measured`, `lagged` or `ik_target`; anything else fails configure/activate |
| `lag_time_constant_s` | double | `0.008` | `lagged`: first-order low-pass of the measured position, q_out += a (q_meas - q_out) with a = 1 - exp(-dt/tau) (a = 1 at tau 0), the exact discretisation at any dt. Drake's IIR (pole 0.8816 per 1 ms tick) is tau 7.94 ms |
| `target_topic` | string | `target_pose` | `ik_target`: geometry_msgs/PoseStamped, relative to the controller's namespace, so `/lbr/target_pose`, the topic crisp's `cartesian_impedance_controller` reads. Subscriber only. Re-subscribed at activation only when it changed. While the topic has more than one publisher every target is ignored (with a throttled warning), as crisp ignores it, so the output goes stale |
| `end_effector_frame` | string | `lbr_link_ee` | `ik_target`: must equal crisp's `end_effector_frame` (controllers.yaml `lbr_link_ee`; controllers_umi.yaml `lbr_umi_camera`) |
| `base_frame` | string | `lbr_link_0` | frame the target is expressed in. crisp has no base frame: it reads the target in the model's world frame. So base_frame must coincide with the world (it does for lbr_ros2_control's iiwa14.xacro); if it is offset, `ik_target` has no kinematics and falls back to `measured`, with the offset in the error |
| `target_filter` | double | `0.9` | EMA weight on the previous filtered target per update, as crisp's `filter.target_pose`, so both act on the same pose |
| `target_timeout_s` | double | `0.1` | no new target for this long -> stale: the output returns to `measured` (rate-limited) until a target arrives again, then IK restarts from the measured position |
| `ik_damping` | double | `0.01` | damped-least-squares lambda |
| `nullspace_gain` | double | `0.05` | per-update pull of q_ik toward the posture at activation, projected into J's nullspace |
| `joint_limit_margin_rad` | double | `0.05` | q_ik is held this far inside the URDF position limits. A margin that leaves no range fails `ik_target`'s activation; the other modes do not use it |
| `max_joint_step_rad` | double[7] | `[0.004]*7` | per-update limit on the change of the OUTPUT (2 rad/s at 500 Hz); applied in every mode except `measured`. Gives way to the gap clamp (step 3). A scalar in a YAML override is read for every joint, but then the parameter is a double and stays one (rclcpp fixes the type from the file), so the shipped YAML gives 7 values |
| `sunrise_joint_stiffness` | double[7] | `[200]*7` | what the running Sunrise app uses; NOT read from the robot (FRI does not report it). Must match the app |
| `max_spring_torque_nm` | double[7] | `[10,10,10,8,5,5,3]` | per joint, |q_out - q_meas| <= max_spring_torque / sunrise_joint_stiffness. At 200 Nm/rad: 0.05 rad on A1-A3 |

Law in `ik_target`, per update (dt from the controller manager, 500 Hz on this cell):

1. Read the newest target (realtime buffer; none while the topic has more than one publisher);
   filter it: position lerp, orientation slerp, weight `target_filter` on the previous filtered
   value (the first target after activation, or after a fallback, initialises it).
2. Differential IK integrated on the controller's own `q_ik`, never on the measured position:
   `e = [p_target - p(q_ik); log3(R_target R(q_ik)^T)]` in base axes (pinocchio
   LOCAL_WORLD_ALIGNED Jacobian of `end_effector_frame` at q_ik),
   `dq = J^T (J J^T + lambda^2 I)^-1 e + (I - J^+ J) * nullspace_gain * (q_posture - q_ik)`,
   `q_ik += dq`, then clamp q_ik inside the URDF limits minus the margin.
   Integrating on the measured position would reproduce today's behaviour.
3. Output: `q_out = clamp(q_ik, q_meas - gap, q_meas + gap)` per joint (gap from the stiffness and
   torque parameters), then `|q_out - q_out_prev| <= max_joint_step_rad`, then the gap clamp
   again. The gap has the last word: when the arm itself moves faster than the step limit, a
   step-limited output would trail it by more than the gap, so the step limit gives way
   (`gap_clamped` and `step_clamped` both 1). The same order holds in `lagged` and in the
   measured fallback.
4. Any non-finite value, a missing frame, a base_frame offset from the world, or a stale target
   -> the measured position (rate-limited from the previous output, within the gap), with a
   throttled warning. A missing or non-finite measured state is an ERROR, as today.

On activation in every mode: q_out = q_ik = q_posture = the measured position, the filtered target
and the target buffer are cleared (a target published before this activation is never used), so
the first output equals today's.

On deactivation in every mode the measured position is written into the position command. ARMED
-> REST activates `joint_trajectory_controller` in the same switch, and it holds the command it
finds (`set_last_command_interface_value_as_state_on_activation`, default true). Under `ik_target`
the last q_out can sit up to the gap away from the arm, and REST would hold that spring torque
indefinitely; with the measured position REST holds the arm where it stands, as before.

Diagnostics: a realtime publisher `~/state` (std_msgs/Float64MultiArray, every update), layout
`[mode (0 measured, 1 lagged, 2 ik_target), stale (0/1), q_meas[7], q_ik[7], q_out[7],
gap_clamped[7] (0/1), step_clamped[7] (0/1)]` = 37 values (2 + 5 x 7); `layout.dim[0].label`
names it.

## Safety notes

- lbr_fri_ros2's command guard checks position limits, measured velocity and torque, but nothing
  checks |q_cmd - q|. Under `ik_target` that gap times Sunrise's stiffness IS a joint torque the
  robot applies, on top of crisp's overlay. The gap clamp is the guard for it; it must hold for
  any target, including a jump.
- Raising Sunrise's stiffness (the second Sunrise app, below) multiplies that torque; change
  `sunrise_joint_stiffness` in the same step, and only after the safety review the commissioning
  protocol requires. A `sunrise_joint_stiffness` below the app's is as bad: it widens the gap.
  `scripts/position_source.py set` refuses, for `lagged` and `ik_target`, any value looser than
  the defaults (a stiffness other than 200, a larger `max_spring_torque_nm`, a wider gap, a larger
  `max_joint_step_rad`) unless given `--limit-change-reviewed`, which prints the resulting gap and
  torque before setting. `set measured` always works.
- Only `cartesian_impedance_controller` tracks `/lbr/target_pose`. `kuka_crisp.safe_switch`
  refuses to arm, or switch while ARMED to, any other overlay (`joint_impedance_controller`)
  unless the passthrough is in `measured`: the parameter at REST, and also the mode `~/state`
  reports while ARMED. Under `ik_target`, Sunrise's spring would otherwise pull toward the IK of a
  pose that overlay is not following.
- Known behaviour, `ik_target`: the nullspace pull targets the posture at activation, while the
  arm's elbow is set by crisp's own nullspace and by friction. A persistent swivel difference
  larger than the gap shows as `gap_clamped` on A1/A3 (A5/A7) for the whole session while the
  pose error is small: a steady spring torque along the swivel within the gap bound. Stop
  criterion in the cell test below.
- Known difference from crisp, `ik_target`: crisp starts its filtered target at FK(q_meas) on
  activation, never goes stale and ramps every new target in from its previous desired pose. This
  filter starts on the first target after activation or after a fallback, so after a pause
  longer than `target_timeout_s` the IK jumps to a new target while crisp ramps (about 20 ms at
  0.9 and 500 Hz). Within the gap and step bounds; it adds a little overlay torque at the first
  target of each replay.
- Never copy Drake's provisioning: its README presses Install in StationSetup.cat (wipes and
  reboots the cabinet) and unchecks External EMERGENCY STOP, Operator Protection and Safety Stop
  in the Customer PSM. Applications go in by syncing into the project loaded from the cabinet
  (docs/sunrise_setup.md).
- yubi's teleop and policy sessions assume today's behaviour; they should refuse or warn when
  `position_source` is not `measured` until a mode is validated (follow-up in the yubi repo).

## Sunrise side (second app, optional, for stiffness above 200)

`sunrise/LBRServerJointImpedance.java`, package `lbr_fri_ros2`, installed BESIDE the stock
`LBRServer` by a Workbench sync (never a StationSetup install). Same lifecycle and dialogs as
LBRServer (send period, client IP), control mode fixed to joint impedance and command mode to
TORQUE, plus two dialogs: stiffness preset (200 on all joints = today; 500 on all; Drake's
1000/1000/1000/500/500/500/500) and damping preset (0.3 = this cabinet today, if the cabinet's
LBRServer sets it, see docs/sunrise_setup.md; 0.7 = KUKA
default; 1.0 = Drake). Logs what it chose. Not compiled here: there is no Sunrise SDK on Linux.

## Plan and gates

1. Implement the three modes, unit-test the core (no ROS), test end to end on the mock
   (`pixi run -e jazzy mock-loopback`, ROS_DOMAIN_ID 77: never a mock on the shared LAN).
   Gate: on the mock the arm follows the IK of published targets; every clamp and the stale
   fallback fire in a test; `measured` is bit-identical to the old controller.
2. On the cell at today's 200 Nm/rad, three-way: a `track_tune.py steps` recording (1, 2, 3 mm;
   0.5, 2 deg; it moves nothing) and the yubi lead_test recordings, each run with
   `track_tune.py replay` under each `position_source`, crisp at today's gains, the mode switched
   at REST with `scripts/position_source.py set <mode>`. Gate: `ik_target` answers smaller steps with less
   lag and less overlay torque than `measured`; otherwise stop.
3. The second Sunrise app with presets, safety review, repeat 2 at each preset.
4. Lower crisp's gains toward feedforward (the velocity lead, planned-velocity friction).

## Cell test (gate 2)

The three-way comparison of plan step 2, at today's 200 Nm/rad (stock `LBRServer`, so
`sunrise_joint_stiffness` stays at its default) and crisp at the policy gains. Gate 1 (the mock)
must have passed first. Run it on the cell host, in this repository's checkout with this branch
built (`pixi run -e jazzy build`), in bash (`bash` first in a fish terminal), with the operator
on the E-stop. Each `replay` asks for `go`; nothing else here moves the arm.

**Before.** The lead_test recordings are flange (`lbr_link_ee`) targets from the 10-22 rollout,
so the stack is `pixi run -e jazzy hardware` (`tool:=none`), never `hardware-umi`. If they are
not on the cell yet, from the workstation:
`rsync -a ~/yubi_data/track/lead_test/ iris-hyundai:yubi_data/track/lead_test/`.

```bash
cd ~/github.com/lvjonok/hardware/pixi_kuka_ros2
OUT=~/yubi_data/track/position_source/$(date +%F)    # one directory per session
LT=~/yubi_data/track/lead_test
G=$LT/policy_cap04.yaml                              # policy gains + track_tune's 40 mm cap
mkdir -p $OUT

make -C ~/github.com/lvjonok/yubi/deploy rollout-home   # MOVES THE ARM: joint home, left at REST
pixi run -e jazzy position-source status    # "cell: REST", position_source declared (an old build lists none)

# The steps, once, about where the arm stands (reads /lbr/current_pose, moves nothing):
pixi run -e jazzy python scripts/track_tune.py steps $OUT/steps.jsonl --mm 1 2 3 --deg 0.5 2
```

**Each mode**, in this order: `measured` (today's baseline), `lagged`, `ik_target`. The
result directories are `$OUT/<recording>/<mode>`, so that `rescore.py`, which labels a log by
its parent directory, prints the mode.

```bash
MODE=measured                                # then MODE=lagged, then MODE=ik_target
pixi run -e jazzy position-source set $MODE  # refuses unless at REST
make -C ~/github.com/lvjonok/yubi/deploy cell-arm   # REST -> ARMED (kuka_cell.py: refuses a
                                             # target publisher, rests again if the arm moves)
pixi run -e jazzy position-source status     # "running (~/state): $MODE"; stop if not
mkdir -p $OUT/steps/$MODE

# second terminal, for the steps replay only (overlay torque and clamps):
#   pixi run -e jazzy ros2 bag record -s mcap -o $OUT/steps/$MODE/bag \
#       /lbr/lbr_state /lbr/fri_position_passthrough_controller/state /rosout
pixi run -e jazzy python scripts/track_tune.py replay $OUT/steps.jsonl \
    --gains $G --restore $G --out $OUT/steps/$MODE
#   ^C the bag

for R in orig lead_dk lead_meas; do
  pixi run -e jazzy python scripts/track_tune.py replay $LT/r143719/$R.jsonl \
      --gains $G --restore $G --out $OUT/$R/$MODE
done

make -C ~/github.com/lvjonok/yubi/deploy cell-rest  # ARMED -> REST, before the next MODE
```

Without the yubi deploy preset, `pixi run -e jazzy session --switch cartesian_impedance_controller`
arms and `pixi run -e jazzy session --switch zero_effort_controller` rests (the same STRICT
switches, without kuka_cell's publisher refusal and arming watch).

Between replays nothing publishes `/lbr/target_pose`, so under `ik_target` the passthrough goes
stale after `target_timeout_s` and writes the measured position; that is the designed fallback.
**Stop, REST and `set measured`** on an aborted replay under `lagged` or `ik_target`, an FRI drop,
a `status` that shows a joint gap-clamped while the arm is still, or A1/A3 gap-clamped through
most of a replay whose pose error stays small (the nullspace posture, not the target, set it).

**After, always**, before any yubi teleop or policy session (they assume `measured` and do not
check yet):

```bash
pixi run -e jazzy position-source set measured
pixi run -e jazzy position-source status     # position_source measured; cell: REST
```

**Compare.** Steps, from each replay's `summary.json` (`trials[0]`; track_tune's `score` and
`fine`): `small_delay_p50_ms` / `small_delay_p90_ms` (target to arm at half of each 1-3 mm step),
`small_settle_p50_mm` (the error 0.5 s after a step), `hold_p50_mm` / `hold_p95_mm` and
`hold_p50_deg` / `hold_p95_deg` (the offset left once the target has been still 1 s: friction),
`pos_lag_ms` / `rot_lag_ms`, `peak_joint_speed_frac` and `aborted`:

```bash
for M in measured lagged ik_target; do
  pixi run -e jazzy python -c 'import json, sys
t = json.load(open(sys.argv[1]))["trials"][0]
keys = ("small_delay_p50_ms", "small_delay_p90_ms", "small_settle_p50_mm", "hold_p50_mm",
        "hold_p95_mm", "hold_p50_deg", "hold_p95_deg", "pos_lag_ms", "rot_lag_ms",
        "peak_joint_speed_frac", "aborted")
print(f"{sys.argv[2]:10s}", "  ".join(f"{k} {t.get(k)!s:.6}" for k in keys))' \
    $OUT/steps/$M/summary.json $M
done
```

Overlay torque per joint over the steps replay (FRI's `commanded_torque`), the torque Sunrise's
spring added (`(commanded - measured position) x 200`), and how often each clamp cut the output:

```bash
pixi run -e jazzy python scripts/overlay_torque.py $OUT/steps/{measured,lagged,ik_target}/bag
```

The lead_test replays: **not** their `summary.json`, which scores a lead run against its own,
shifted, target. `rescore.py` scores every log against the original timing (`orig.jsonl`):
position and rotation lag, RMS before and after it, and the share of each 1 s window's
commanded travel the arm covered:

```bash
for R in orig lead_dk lead_meas; do
  echo "== $R"
  pixi run -e jazzy python $LT/rescore.py $LT/r143719/orig.jsonl \
      $OUT/$R/{measured,lagged,ik_target}/policy_cap04.jsonl
done
```

Gate: `ik_target` answers the steps with less lag (`small_delay_p50_ms`, `pos_lag_ms`,
`rot_lag_ms`) and less overlay torque (overlay rms on A1-A4) than `measured`, with no abort;
otherwise stop. `lagged` separates what the friction compensation alone buys. A joint
gap-clamped most of the time means the 0.05 rad gap, not the IK, set the answer. Copy the results
to the workstation with `make -C deploy rollout-pull` there (it rsyncs `yubi_data/track/`).

## Changes after review

Where the implementation now differs from the first draft of this specification:

- Step 3: the gap clamp is applied again after the step clamp (the draft ended on the step clamp,
  which let |q_out - q_meas| exceed the gap whenever the arm moved faster than the step limit).
- `lagged`: a = 1 - exp(-dt/tau) instead of dt/(tau+dt), which lagged about 12 % more than
  Drake's filter at 2 ms.
- `max_joint_step_rad` is a double[7]; the draft's dynamically typed "double[7] or double" never
  took effect with a YAML override.
- Targets are ignored while `/lbr/target_pose` has more than one publisher, as crisp ignores them.
- `base_frame` must coincide with the model's world, where crisp reads the target.
- On deactivation the measured position is written into the position command.
- A parameter error fails activation with FAILURE (was ERROR); the joint-limit margin is checked
  only in `ik_target`.
- `position_source.py set` refuses looser limits without `--limit-change-reviewed`;
  `kuka_crisp.safe_switch` refuses non-Cartesian overlays over a passthrough not in `measured`.
