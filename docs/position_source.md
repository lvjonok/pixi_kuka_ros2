# What the FRI position field carries under the torque overlay

In `TORQUE` client command mode FRI sends a joint position and a torque every cycle. Sunrise runs its joint
impedance spring around that position (200 Nm/rad on every joint in `LBRServer`) and adds the torque on
top. `fri_position_passthrough_controller` writes the position, and its parameter `position_source` says
which one.

| Mode | The position field carries | Sunrise's spring |
| --- | --- | --- |
| `measured` (default) | the measured position | holds nothing; crisp's overlay holds the arm |
| `lagged` | a first-order low-pass of the measured position | a small gap that trails any motion |
| `ik_target` | the differential IK of crisp's target, `/lbr/target_pose` | pulls toward the target as well, within the gap clamp |

`lagged` is what Drake's driver sends in its torque-only mode. Its code says that KUKA's friction observer
acts only on a position difference (`kuka_driver.cc` in drake-iiwa-driver).

**`lagged` and `ik_target` have not been checked on the arm.** They pass the bench check on mock hardware
(`check-mock`) and the gtests of the law.

## Parameters

All are parameters of `fri_position_passthrough_controller`. They are read when the controller is
configured and again at every activation, so a change takes effect at the next arm (REST to ARMED), and
every activation logs the mode it took.

A value that fails validation fails the activation, and the switch from REST with it. The controller
manager has deactivated `joint_trajectory_controller` and `zero_effort_controller` by then and does not
activate them again: no controller owns the position or the torque command, and `crisp_session.py` refuses
to switch. `position-source set` refuses such values for this reason; a value set another way gets past it.
Back to REST by hand:

```bash
pixi run -e jazzy ros2 control switch_controllers -c /lbr/controller_manager --strict \
  --activate joint_trajectory_controller zero_effort_controller
```

| Parameter | Type | Default | Meaning |
| --- | --- | --- | --- |
| `position_source` | string | `measured` | `measured`, `lagged` or `ik_target`. |
| `lag_time_constant_s` | double | `0.008` | `lagged`: q_out += a (q_meas - q_out) with a = 1 - exp(-dt/tau); a = 1 at tau 0. Drake's filter (pole 0.8816 per 1 ms) is tau 7.94 ms. |
| `target_topic` | string | `target_pose` | `ik_target`: a `geometry_msgs/PoseStamped` topic, relative to the controller's namespace, so `/lbr/target_pose`, the one `cartesian_impedance_controller` reads. While the topic has more than one publisher every target is ignored, as crisp ignores it, and the output goes stale. |
| `end_effector_frame` | string | `lbr_link_ee` | `ik_target`: must be crisp's `end_effector_frame`. `config/controllers_umi.yaml` sets both to `lbr_umi_camera`. |
| `base_frame` | string | `lbr_link_0` | The frame the target is given in. crisp reads the target in the model's world frame, so `base_frame` must coincide with the world, as it does in `iiwa14.xacro`. If it does not, `ik_target` has no kinematics and writes the measured position. |
| `target_filter` | double | `0.9` | Weight of the previous filtered target per update, as crisp's `filter.target_pose`. |
| `target_timeout_s` | double | `0.1` | With no new target for this long the target is stale: the output goes back to the measured position until a target arrives, and the IK then starts again from the measured position. |
| `ik_damping` | double | `0.01` | The lambda of the damped least squares. |
| `nullspace_gain` | double | `0.05` | Pull of q_ik toward the posture at activation per update, in the nullspace of the Jacobian. |
| `joint_limit_margin_rad` | double | `0.05` | q_ik stays this far inside the description's position limits. A margin that leaves no range fails the activation in `ik_target`. |
| `max_joint_step_rad` | double[7] | `0.004` each | Limit on the change of the output per update, in every mode but `measured`: 2 rad/s at 500 Hz. It gives way to the gap clamp. |
| `sunrise_joint_stiffness` | double[7] | `200` each | The stiffness the running Sunrise application uses. It is not read from the robot: FRI does not report it. It must match the application. |
| `max_spring_torque_nm` | double[7] | `10, 10, 10, 8, 5, 5, 3` | Per joint, the output stays within `max_spring_torque_nm / sunrise_joint_stiffness` of the measured position: the gap. At 200 Nm/rad that is 0.05 rad on A1 to A3. |

## The law

Per update, with dt from the controller manager (2 ms):

1. `ik_target` reads the newest target and filters it: position by linear interpolation, orientation by
   slerp, with weight `target_filter` on the previous filtered value. The first target after an activation
   or after a stale period starts the filter.
2. `ik_target` integrates a differential IK on its own `q_ik`, never on the measured position:
   `e = [p_target - p(q_ik); log3(R_target R(q_ik)^T)]` in base axes, with pinocchio's
   `LOCAL_WORLD_ALIGNED` Jacobian of `end_effector_frame` at `q_ik`;
   `dq = J^T (J J^T + lambda^2 I)^-1 e + (I - J^+ J) nullspace_gain (q_posture - q_ik)`;
   `q_ik += dq`, then `q_ik` is clamped inside the position limits less the margin.
3. The output is the proposal (`q_ik`, or the low-pass in `lagged`) clamped per joint to the gap around the
   measured position, then to `max_joint_step_rad` from the previous output, then to the gap again. The gap
   has the last word: when the arm moves faster than the step limit, the step limit gives way, and
   `gap_clamped` and `step_clamped` are both set.
4. On a value that is not finite, a missing frame, a `base_frame` away from the world or a stale target,
   the output is the measured position, reached at the step limit and within the gap, with a throttled
   warning.

A joint whose position state is not finite keeps its last finite value, and the error is logged.

At activation, in every mode, `q_out`, `q_ik` and the posture are the measured position, and the filtered
target and the target buffer are cleared: a target published before the activation is never used.

At deactivation, in every mode, the measured position is written into the position command. ARMED to REST
activates `joint_trajectory_controller` in the same switch, and it holds the command it finds. Under
`ik_target` the last output can be up to the gap from the arm, and REST would hold that spring torque;
with the measured position REST holds the arm where it stands.

## State

`/lbr/fri_position_passthrough_controller/state` is a `std_msgs/Float64MultiArray`, published every update
while the controller is active. `layout.dim[0].label` names its 37 values:

```text
[0]      mode             0 measured, 1 lagged, 2 ik_target
[1]      stale            1 while ik_target has no fresh target (the output is the measured position)
[2:9]    q_meas[7]
[9:16]   q_ik[7]
[16:23]  q_out[7]         what went into the position command
[23:30]  gap_clamped[7]   1 where the gap clamp cut q_out in this update
[30:37]  step_clamped[7]  1 where max_joint_step_rad cut it
```

## Setting it

```bash
pixi run -e jazzy position-source status
pixi run -e jazzy position-source set lagged
pixi run -e jazzy position-source set ik_target --param target_timeout_s=0.2
pixi run -e jazzy position-source set measured
```

`status` prints the parameters, the cell's state and, while the passthrough is active, the mode it runs
and the joints each clamp cuts. `set` moves nothing and switches no controller. It

- refuses unless the cell is at REST: the passthrough loaded and inactive, `joint_trajectory_controller`
  and `zero_effort_controller` active;
- refuses a value the controller would fail its activation on;
- for `ik_target`, refuses frames or a target topic other than `cartesian_impedance_controller`'s, and
  warns when `target_filter` differs from its `filter.target_pose`;
- for `lagged` and `ik_target`, refuses a value that loosens the bound on Sunrise's spring torque: a
  stiffness other than 200, a `max_spring_torque_nm` above the defaults, a wider gap, a
  `max_joint_step_rad` above 0.004. `--limit-change-reviewed` allows it and prints the resulting gap and
  torque first. `set measured` always works;
- sets the parameters in one atomic call and reads them back.

A relaunch of the stack returns to `config/controllers.yaml`. `position-source-mock` is the same tool on
domain 77 on loopback.

## Safety

- lbr_fri_ros2's command guard checks position limits, measured velocity and torque. Nothing checks
  `|q_cmd - q|`. Under `ik_target` that gap times Sunrise's stiffness is a joint torque the robot applies
  on top of crisp's overlay. The gap clamp is the only bound on it, and it is software on this PC.
- A Sunrise application with a higher stiffness multiplies that torque. Change `sunrise_joint_stiffness`
  in the same step, and only after the review [`commissioning_protocol.md`](commissioning_protocol.md)
  asks for. A `sunrise_joint_stiffness` below the application's is as bad: it widens the gap.
- Only `cartesian_impedance_controller` follows `/lbr/target_pose`. `kuka_crisp.safe_switch` refuses to arm
  `joint_impedance_controller`, or to switch to it while ARMED, unless the passthrough is in `measured`:
  the parameter at REST, and also the running mode while ARMED.
- In `ik_target` the nullspace pull goes toward the posture at activation, while the arm's elbow is set by
  crisp's own nullspace term and by friction. A lasting difference larger than the gap shows as
  `gap_clamped` on A1 and A3, or A5 and A7, while the pose error is small: a steady spring torque within
  the gap bound.
- crisp starts its filtered target at the measured pose and ramps every new target in. The passthrough's
  filter starts on the first target after an activation or a stale period, so after a pause longer than
  `target_timeout_s` its IK jumps to the new target while crisp ramps over about 20 ms. The difference
  stays within the gap and step bounds.

[`sunrise/LBRServerJointImpedance.java`](../sunrise/LBRServerJointImpedance.java) is a Sunrise application
with stiffness and damping presets above `LBRServer`'s.
[`sunrise_setup.md`](sunrise_setup.md#second-app-lbrserverjointimpedance-stiffness-and-damping-presets)
