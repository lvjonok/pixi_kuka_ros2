# What the FRI position field carries under the torque overlay

In `TORQUE` client command mode FRI sends a joint position and a torque every cycle. Sunrise runs its joint
impedance spring around that position and adds the torque on top.
`fri_position_passthrough_controller` writes the position: the differential IK of crisp's Cartesian target,
`/lbr/target_pose`, held within a bounded gap of the measured position. While the cell is ARMED, Sunrise's
spring so pulls every joint toward the target together with crisp's torque, with at most
`max_spring_torque_nm`.

Without a target it can use, the position is the measured one and the spring holds nothing. That is the
case

- from an activation until the first target arrives;
- when the newest target is older than `target_timeout_s`;
- while `/lbr/target_pose` has a second publisher: every target is then ignored, as crisp ignores it, and
  the last one taken goes stale;
- when the kinematics are missing: no robot description, a frame that is not in it, or a `base_frame`
  that is not the model's world.

A target that is not finite is dropped, and the last good one stays in use until it is stale.

## The Sunrise application and `sunrise_joint_stiffness`

FRI does not report the stiffness of Sunrise's spring, so the controller takes it from the parameter
`sunrise_joint_stiffness`. It holds the position within `max_spring_torque_nm / sunrise_joint_stiffness`
of the measured one, the gap. The torque the spring then applies is the gap times the stiffness the
application really runs.

**`sunrise_joint_stiffness` is the stiffness of the application that runs. A parameter below the
application's is the unsafe side:** the spring exceeds `max_spring_torque_nm` by the ratio of the two. A
parameter above the application's gives a weaker spring: the safe side.

| Application on the cabinet | Its stiffness, Nm/rad | With the default parameters the spring pulls with at most |
| --- | --- | --- |
| [`FRITorqueDriver`](../sunrise/FRITorqueDriver.java), the one the cell runs | 1000 on A1 to A3, 500 on A4 to A7 | `max_spring_torque_nm`: 5, 5, 5, 4, 2.5, 2.5, 1.5 Nm |
| `LBRServer` of lbr_fri_ros2_stack | 200 on every joint | a fifth of that on A1 to A3 and two fifths on A4 to A7: 1, 1, 1, 1.6, 1, 1, 0.6 Nm |

The default `sunrise_joint_stiffness` is that of `FRITorqueDriver`. With a parameter of 200 under its 1000
the spring would pull with five times `max_spring_torque_nm`.

The parameter is set in `config/controllers.yaml`. At REST it can also be set on the running stack, and
holds until the stack is launched again:

```bash
pixi run -e jazzy ros2 param set /lbr/fri_position_passthrough_controller sunrise_joint_stiffness \
  "[1000.0, 1000.0, 1000.0, 500.0, 500.0, 500.0, 500.0]"
```

## Parameters

All are parameters of `fri_position_passthrough_controller`. They are read when the controller is
configured and again at every activation, so a change takes effect at the next arm (REST to ARMED). An
activation logs the frames, the topic, the filter, the timeout, the damping, the nullspace gain and the
margin it took; the three per-joint parameters are read with `ros2 param get`.

A value that fails validation fails the activation, and the switch from REST with it. The controller
manager has deactivated `joint_trajectory_controller` and `zero_effort_controller` by then and does not
activate them again: no controller owns the position or the torque command, and `crisp_session.py` refuses
to switch. After the value is put right, REST comes back by the strict switch by hand of
[the README](../README.md#bringing-up-the-real-arm), step 7.

| Parameter | Type | Default | Meaning |
| --- | --- | --- | --- |
| `position_source` | string | `ik_target` | Its only value; any other fails the activation. Tools that record a run read it by name. |
| `target_topic` | string | `target_pose` | A `geometry_msgs/PoseStamped` topic, relative to the controller's namespace, so `/lbr/target_pose`, the one `cartesian_impedance_controller` reads. |
| `end_effector_frame` | string | `lbr_link_ee` | The frame the IK solves for. It must be crisp's `end_effector_frame`. `config/controllers_umi.yaml` sets both to `lbr_umi_camera`. |
| `base_frame` | string | `lbr_link_0` | The frame the target is given in. crisp reads the target in the model's world frame, so `base_frame` must coincide with the world, as it does in `iiwa14.xacro`. |
| `target_filter` | double | `0.9` | Weight of the previous filtered target per update, as crisp's `filter.target_pose`. |
| `target_timeout_s` | double | `0.1` | With no new target for this long the target is stale: the output goes back to the measured position until a target arrives, and the IK then starts again from the measured position. |
| `ik_damping` | double | `0.01` | The lambda of the damped least squares. |
| `nullspace_gain` | double | `0.05` | Pull of q_ik toward the posture at activation per update, in the nullspace of the Jacobian. |
| `joint_limit_margin_rad` | double | `0.05` | q_ik stays this far inside the description's position limits. A margin that leaves no range fails the activation. |
| `max_joint_step_rad` | double[7] | `0.004` each | Limit on the change of the output per update: 2 rad/s at 500 Hz. It gives way to the gap clamp. |
| `sunrise_joint_stiffness` | double[7] | `1000, 1000, 1000, 500, 500, 500, 500` | The stiffness of the Sunrise application that runs. See above. |
| `max_spring_torque_nm` | double[7] | `5, 5, 5, 4, 2.5, 2.5, 1.5` | Per joint, the output stays within `max_spring_torque_nm / sunrise_joint_stiffness` of the measured position: 0.005 rad on A1 to A3, 0.008 on A4, 0.005 on A5 and A6, 0.003 on A7. |

The defaults are also the values of `config/controllers.yaml`.

## The law

Per update, with dt from the controller manager (2 ms):

1. The newest target is filtered: position by linear interpolation, orientation by slerp, with weight
   `target_filter` on the previous filtered value. The first target after an activation or after a stale
   period starts the filter.
2. A differential IK is integrated on its own `q_ik`, never on the measured position:
   `e = [p_target - p(q_ik); log3(R_target R(q_ik)^T)]` in base axes, with pinocchio's
   `LOCAL_WORLD_ALIGNED` Jacobian of `end_effector_frame` at `q_ik`;
   `dq = J^T (J J^T + lambda^2 I)^-1 e + (I - J^+ J) nullspace_gain (q_posture - q_ik)`;
   `q_ik += dq`, then `q_ik` is clamped inside the position limits less the margin.
3. The output is `q_ik` clamped per joint to the gap around the measured position, then to
   `max_joint_step_rad` from the previous output, then to the gap again. The gap has the last word: when
   the arm moves faster than the step limit, the step limit gives way, and `gap_clamped` and
   `step_clamped` are both set.
4. On a stale target, missing kinematics, a period that is not positive and finite or an IK result that
   is not finite, the output is the measured position, reached at the step limit and within the gap, with
   a throttled warning.

A joint whose position state is not finite keeps its last finite value, and the error is logged.

At activation `q_out`, `q_ik` and the posture are the measured position, and the filtered target and the
target buffer are cleared: a target published before the activation is never used. Arm with the arm near
the home: the IK starts where the arm is, but far from the home its joints drift away from the arm's while
it solves the target. `crisp_session.py --home` moves there.

At deactivation the measured position is written into the position command. ARMED to REST activates
`joint_trajectory_controller` in the same switch, and it holds the command it finds. The last output can be
up to the gap from the arm, and REST would hold that spring torque; with the measured position REST holds
the arm where it stands.

## State

`/lbr/fri_position_passthrough_controller/state` is a `std_msgs/Float64MultiArray`, published every update
while the controller is active. `layout.dim[0].label` names its 37 values:

```text
[0]      mode             always 2
[1]      stale            1 while there is no fresh target (the output is the measured position)
[2:9]    q_meas[7]
[9:16]   q_ik[7]
[16:23]  q_out[7]         what went into the position command
[23:30]  gap_clamped[7]   1 where the gap clamp cut q_out in this update
[30:37]  step_clamped[7]  1 where max_joint_step_rad cut it
```

## Looking at it

```bash
pixi run -e jazzy ros2 param dump /lbr/fri_position_passthrough_controller
pixi run -e jazzy ros2 param get /lbr/fri_position_passthrough_controller sunrise_joint_stiffness
pixi run -e jazzy ros2 topic echo --once /lbr/fri_position_passthrough_controller/state
```

The state is there only while the cell is ARMED. On the mock the same commands take
`ROS_DOMAIN_ID=77 CYCLONEDDS_URI=file://$PWD/config/cyclonedds_loopback.xml` in front.

## Safety

- lbr_fri_ros2's command guard checks position limits, measured velocity and torque. Nothing checks
  `|q_cmd - q|`. That gap times Sunrise's stiffness is a joint torque the robot applies on top of crisp's
  overlay. The gap clamp is the only bound on it, and it is software on this PC.
- An application stiffer than `sunrise_joint_stiffness` multiplies that torque. A change of the
  application's stiffness is a change of this bound: the parameter changes in the same step, at REST, and
  the risk assessment is revisited first
  ([`commissioning_protocol.md`](commissioning_protocol.md#fri-and-torque-control)).
- The spring pulls toward the IK of `/lbr/target_pose`, so the passthrough belongs with the controller
  that follows that topic. `kuka_crisp.safe_switch` arms `cartesian_impedance_controller` and nothing else.
  A strict switch by hand that activates the passthrough with another torque controller is not refused.
- The nullspace pull goes toward the posture at activation, while the arm's elbow is set by crisp's own
  nullspace term and by friction. A lasting difference larger than the gap shows as `gap_clamped` on A1
  and A3, or A5 and A7, while the pose error is small: a steady spring torque within the gap bound.
- crisp starts its filtered target at the measured pose and ramps every new target in. The passthrough's
  filter starts on the first target after an activation or a stale period, so after a pause longer than
  `target_timeout_s` its IK jumps to the new target while crisp ramps over about 20 ms. The difference
  stays within the gap and step bounds.
