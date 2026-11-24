# KUKA iiwa 14 R820 commissioning protocol

The startup procedure for the cell's robot. Complete every safety check here before starting
FRI or activating a crisp torque controller.

## Before you start: record your cell

This protocol is written for a KUKA LBR iiwa 14 R820 (kinematic system `K1`).
Everything below assumes you have already established, and written down, the
values specific to your installation:

| Value | Where to get it |
| --- | --- |
| Sunrise.OS version | smartPAD |
| Sunrise.Workbench version | must match Sunrise.OS |
| FRI client SDK version | the FRI version installed in Sunrise |
| X66 (KLI) address | smartPAD network configuration, or Workbench's connection |
| X66 MAC | ARP, once the address is known |
| ROS workstation address on the robot subnet | your own network setup |
| Robot-facing interface name | `ip -br addr` on the workstation |

Setting up the network is your job, not this workspace's. See
[`sunrise_setup.md`](sunrise_setup.md#network-layout) for the constraints that
arrangement has to satisfy. `172.31.1.147` (cabinet) and `172.31.1.148`
(FRI client) are the KUKA factory defaults and are used as the examples throughout.

FRI runs on UDP `30200`.

### Verify X66 reachability

The cabinet does not answer ICMP echo requests. Verify reachability through a fresh
ARP/neighbour entry instead, substituting your own addresses and interface:

```bash
ping -c1 -W1 -I 172.31.1.148 172.31.1.147 >/dev/null 2>&1   # populates ARP only
ip neigh show 172.31.1.147 dev <nic>
```

Expect a `REACHABLE` entry carrying the MAC you recorded, not a ping reply. Match the
MAC, not just the address: it is what tells the cabinet apart from another host that
has taken the address.

`INCOMPLETE`, `FAILED`, or no entry means the controller is not on the wire. The KLI
comes up late in the Sunrise boot, so re-check once the smartPAD has finished
starting before treating it as a fault, then follow
[Debugging X66 reachability](sunrise_setup.md#debugging-x66-reachability).

## Position-reference warning after startup

After startup the smartPAD can show:

```text
Position sensor not referenced (K1: A1, A2, A3, A4, A5, A6, A7)
```

This is a position-referencing condition, not an instruction to change mastering data.
Do not select **Master**, **Unmaster**, or reset mastering data when this is the only
position-sensor warning.

### Successful recovery

1. Keep FRI and all external ROS control stopped.
2. Confirm there is no separate `Position sensor not mastered` error.
3. Clear the robot's full reachable workspace and keep the E-stop immediately
   available.
4. Confirm that the attached tool and payload match the station configuration.
5. On the smartPAD, open **Applications** and select
   `PositionAndGMSReferencing`.
6. Select the operating mode required by the installed KUKA reference application
   (on this cell: its normal automatic reference sequence).
7. Start the application and follow the smartPAD prompts through the complete A1-A7
   trajectory. Do not interrupt the sequence.
8. Open **Safety -> Status** and confirm that `Position sensor not referenced` has
   cleared for every axis.

If the application is missing, disabled, or forbidden, stop. Synchronize or install the
matching reference application with the correct Sunrise Workbench version and an
authorized commissioning user. Do not substitute manual zero crossing, CRR, or
remastering without following the matching KUKA manual and site safety procedure.

## Axis driven outside its limit

An axis jogged past its limit, for example A4 at **-121 deg** against a **-120 deg** limit,
gives this message on the smartPAD, from the drive rather than from the safety
configuration:

```text
Error in bus interface THIN4MAdapter: hardware limit exceeded, drive 4
```

The message cannot be acknowledged and KRF is not offered in the mode list, which
leaves the axis stuck: T1 enforces the range the axis is already outside of.

**Recovery for this cell: unmaster the offending axis, jog it back inside its range, then
recalibrate.** Unmastering removes the absolute position reference that the range
monitoring is derived from, which is what allows the axis to be moved back.

### This is not the same case as the position-reference warning

The section above forbids selecting **Master**, **Unmaster**, or resetting mastering data.
That prohibition stands, and it applies to its own trigger: a
`Position sensor not referenced` warning after startup with the arm **inside** its
limits. There, unmastering destroys good data to fix a problem that referencing already
fixes.

This section is the opposite situation: an axis is **physically outside** its permitted
range, no acknowledgement is possible, and no mode will move it. Tell the two apart by
the axis angle before touching mastering data. If the axis is inside its limits, do not
unmaster.

### What it costs

- **The arm has no valid absolute position until it is recalibrated.** Nothing may run
  against it in the meantime: no FRI session. `FRITorqueDriver` is stopped on the smartPAD, and
  `pixi run -e jazzy hardware` is not launched.
- Recalibration follows the KUKA procedure for this arm, with the correct equipment and
  an authorized user. Follow the Sunrise.OS 1.17 manual, not this file, for the actual
  steps.
- Afterwards, re-run the A1-A7 position referencing and confirm `Safety -> Status` is
  clear before restarting FRI.
- Record what was unmastered, when, and the recalibration result.

### Prevention

Jogging is the exposure, not the ROS side. `move_to_home` refuses a target outside the
position limits of the robot description, which are 1 deg inside the axis limits. It does
not check the pose it starts from.

## Operating modes and unattended operation

T1 with the enabling switch held is a commissioning mode, not a run mode. The modes
this cabinet offers:

| Mode | Stands for | Enabling switch | Velocity |
| --- | --- | --- | --- |
| T1 | Test 1 | held, plus Start held | reduced, 250 mm/s |
| T2 | Test 2 | held | programmed |
| AUT | Automatic | not required | programmed |
| AUT EXT | Automatic External | not required, external PLC starts it | programmed |

Sunrise also has **KRF** (*Kontrollierte Roboterfahrt*), a recovery mode for jogging
out of a violated safety zone after a safety stop.

Switching mode is a keyswitch on the smartPAD: stop the application, turn the key,
select the mode, turn the key back, remove the key. Never change mode during motion;
it triggers a safety stop.

### The stop behaviour changes in AUT, and not in the obvious way

The red E-STOP is safety-rated and identical in every mode. The grey Stop key is an
application stop and is not a protective device in any mode.

What changes is that **T1's passive stop disappears**. In T1, releasing the enabling
switch drops FRI out of `COMMANDING_ACTIVE` immediately. In AUT nothing is held, so
that reflex does nothing and every stop must be actively reached for. In AUT the
safety configuration carries the weight that a thumb carries in T1.

The smartPAD's E-STOP goes away with the smartPAD. Before running in AUT, wire a
fixed external emergency stop into the cabinet safety interface, and keep the
smartPAD within reach rather than stowed.

### Operator safety blocks AUT on this cell

With the customer PSM table active, selecting AUT gives:

```text
Operator safety is active
user PSM table row 2, input signal operating mode with high speed
```

This is the stock safety configuration behaving correctly, not a fault. Sunrise
enforces a **PSM** table (Permanent Safety Monitoring; its state-dependent sibling is
**ESM**) whose rows combine **AMFs** (Atomic Monitoring Functions) with a stop
reaction. Row 2 requires the operator-safety input, that is the guard interlock a
fence door switch or light curtain closes, whenever an operating mode with high speed
is selected. This cell has no guard wired, so the input reads open and the row fires.

The message names the **customer** PSM table. Sunrise keeps two layers: KUKA's default
safety configuration, which is fixed and cannot be edited or switched off, and the
customer PSM/ESM tables, which the project owner defines. The operator-safety row is a
customer row, inherited from the KUKA project template rather than derived from this
cell.

Three legitimate routes:

1. **Wire a real guard.** Fence the cell, interlock the door, wire it to the
   operator-safety input. AUT then works as designed and the safety config barely
   changes.
2. **Deactivate the customer PSM table.** Removes the row that blocks AUT, leaving only
   KUKA's default safety configuration. Fast, reversible, and what this cell runs with.
   It buys AUT at the cost of every customer-defined monitoring function. See
   [Deactivating the customer PSM to reach AUT](#deactivating-the-customer-psm-to-reach-aut).
3. **Reconfigure for collaborative operation.** Replace that row with Power and Force
   Limiting monitoring (ISO/TS 15066), so the arm is provably safe to be near without a fence.
   The cell has no such configuration. Its limit values are measured on the cell, with the
   actual tool and payload at the actual speeds, and are not copied from any document.

**Never bridge or jumper the operator-safety input.** It tells the cabinet a guard is
closed when nothing is guarding anything, in the one mode where nothing is holding a
deadman. That is defeating a protective device. Route 2 is not this: it is a declared
configuration change, made in the safety editor, that changes the recorded checksum and
leaves the cabinet honestly reporting that no guard is monitored. Jumpering leaves the
cabinet reporting a guard that does not exist.

## Deactivating the customer PSM to reach AUT

This is the configuration the cell runs with.

Deactivating the customer PSM table clears
`Operator safety is active / user PSM table row 2` and lets the keyswitch select AUT.

### What this does and does not remove

Still enforced, from KUKA's default safety configuration, which this change cannot
touch:

- E-STOP on the smartPAD and on the external emergency-stop circuit
- The enabling device in T1 and T2
- The T1 reduced-velocity limit, 250 mm/s
- Operating-mode selection and the safe-state monitoring behind it
- Standstill monitoring on a safety stop

Removed, and this is the whole cost:

- **Every row of the customer PSM table, not only row 2.** Deactivation is per table,
  not per row. Any customer workspace monitoring, axis range limit, velocity limit, or
  tool-dependent row in that table stops being enforced at the same time.

So capture the table before deactivating it. If the table holds rows this cell actually
depends on, delete row 2 alone and leave the table active instead; the result for AUT is
the same and nothing else is lost.

In AUT, with the customer table deactivated, the arm's only protective device is the
E-STOP, and nothing is held in a hand. Wire the fixed external E-STOP before running in
AUT, keep the smartPAD in reach, and keep people out of the reachable workspace while
the robot is enabled. This is a commissioning configuration for an empty cell, not a
configuration to run collaborative work under.

### Capture the baseline before opening the editor

- Safety checksum from `Safety` -> `Status`
- Screenshots of every row of the current PSM and ESM tables
- An archived untouched copy of the Sunrise project outside `%USERPROFILE%\SunriseWorkspace`
  (`sunrise_setup.md` step 2 asks for this already; confirm it exists)
- Installed option packages from `StationSetup.cat` -> `Software`

### Procedure

1. Capture the baseline first, per
   [Capture the baseline before opening the editor](#capture-the-baseline-before-opening-the-editor).
   Screenshot **every row** of the customer PSM and ESM tables; this is the only record
   of what deactivation switched off, and the restore path depends on it.
2. Record the current safety checksum from `Safety` -> `Status`.
3. Acquire safety rights: `Safety` -> `Activation`, `Safety maintenance technician`.
4. Open the safety configuration and deactivate the customer PSM table.
5. Activate the safety configuration. The cabinet requires the activation to be
   confirmed and then restarts.
6. Record the **new** safety checksum. It has changed, and it is the new baseline for
   the list [before FRI starts](#before-fri-starts). The old value is not a target.
7. After the restart, re-check `Safety` -> `Status` for
   `Position sensor not referenced`. A safety configuration activation restarts the
   controller, and the A1-A7 referencing must be redone if the warning is back. See
   [Position-reference warning after startup](#position-reference-warning-after-startup).
8. Turn the keyswitch to AUT and confirm the operator-safety message no longer appears.

### Restoring it

Reactivate the customer PSM table in the safety editor, activate the safety
configuration, and confirm the checksum returns to the value recorded in step 2. If it
does not match, the table was edited and not merely deactivated; restore from the
archived untouched project copy instead of trying to reconstruct rows by hand.

### What this does not buy

- **Not a step at the smartPAD before every start.** `FRITorqueDriver` has no dialogs and
  opens FRI sessions by itself ([`sunrise_setup.md`](sunrise_setup.md#starting-it)). In AUT, with
  the application running, a launch of `pixi run -e jazzy hardware` on the PC puts the arm under
  FRI, and arming it is one more command there.
- **Not a collaborative cell.** No force, pressure or velocity limit has been measured or
  configured. Tuning CRISP gains still belongs in T1, where releasing the enabling
  switch stops the arm.

## FRI and torque control

What holds for this stack in every operating mode.

- **Gains are tuned in T1 first, never in AUT.** A wrong gain accelerates the arm. In T1 the
  reduced velocity is monitored and releasing the enabling switch stops the arm.
- **The ROS-side guards are not protective devices.** `external_torque_limit: 2.0` with
  `external_torque_safety_check: true` in `lbr_system_config.yaml`, and LBR-Stack's
  `safe_stop` command guard, are software on a general-purpose Linux box speaking UDP.
  They are useful. They are not safety-rated and must never be credited as a mitigation
  in the risk assessment.
- **Sunrise's joint stiffness is part of the safety picture.** `FRITorqueDriver` runs 1000 Nm/rad
  on A1 to A3 and 500 on A4 to A7 ([`sunrise_setup.md`](sunrise_setup.md#sunrise-joint-stiffness)).
  Lowering it toward zero would leave the arm held only by ROS-side torque commands. A change of
  the stiffness in either direction materially changes the safety picture and requires the risk
  assessment to be revisited. Do not treat it as a tuning tweak.
- **The position command while ARMED is part of it too**
  ([`position_source.md`](position_source.md)). While ARMED the position command is the IK of
  crisp's target and leads the arm, and the gap times Sunrise's stiffness is a joint torque the
  robot applies on top of crisp's overlay. Nothing in lbr_fri_ros2's command guard checks that
  gap; the passthrough's own gap clamp (`max_spring_torque_nm / sunrise_joint_stiffness` per
  joint) is the guard, and it is ROS-side software like the guards above. Raising
  `max_spring_torque_nm` or `max_joint_step_rad` above the defaults, or setting
  `sunrise_joint_stiffness` below the stiffness of the application that runs
  ([the rule](position_source.md#the-sunrise-application-and-sunrise_joint_stiffness)), is a
  limit change, not a tuning step. Nothing in this repo refuses one.
- **Every safety stop ends the FRI session** and needs a relaunch of
  `pixi run -e jazzy hardware`. `FRITorqueDriver` asks nothing for it: while it runs, it opens
  the next session by itself.

## Before FRI starts

The Windows-side installation of `FRITorqueDriver` is a prerequisite and is documented
separately in [`sunrise_setup.md`](sunrise_setup.md).

Do not start `FRITorqueDriver` or `pixi run -e jazzy hardware` until all of these are true. With
`FRITorqueDriver` left running, the launch of `hardware` alone opens the FRI session: the list is
gone through before every launch.

- Position-reference warning is cleared for A1-A7.
- Sunrise.OS and Sunrise.FRI versions are recorded and the client SDK matches.
- The safety configuration checksum matches the recorded baseline. That baseline is the
  value recorded after
  [deactivating the customer PSM](#deactivating-the-customer-psm-to-reach-aut), not the
  pre-FRI-install value. Any *unexplained* change still stops the session.
- Tool/load data are correct for the physical robot.
- X66 resolves to a `REACHABLE` neighbour entry from the ROS workstation, carrying
  the MAC recorded for the cabinet.
- The robot is in T1 for the first commissioning tests.
- The workspace is clear and the E-stop is available.
- The application on the cabinet is `FRITorqueDriver`, and its client address is the ROS
  workstation's on the robot subnet.
- `sunrise_joint_stiffness` in `config/controllers.yaml` is the stiffness of the application that
  runs ([the rule](position_source.md#the-sunrise-application-and-sunrise_joint_stiffness)).

The stack must come up at REST: `joint_trajectory_controller` and `zero_effort_controller`
active, `fri_position_passthrough_controller` and `cartesian_impedance_controller` inactive.

## Moving to the home configuration

The home configuration is `[0, 30, 0, -75, 0, 75, 0]` degrees on A1-A7. Every joint is at
least 45 deg from its axis limit, and the manipulability sqrt(det(J J^T)) is 0.126 there; it
is 0 at the all-zero pose, which is singular. The number is `HOME_DEGREES` in
`scripts/lbr_cell.py`.

### How the move works

The arm is moved by commanding **positions**, not torques. `joint_trajectory_controller`
holds the position command interfaces at REST and runs the move; FRI passes the commanded
positions to Sunrise's joint impedance controller at the stiffness `FRITorqueDriver` sets, 1000
Nm/rad on A1 to A3 and 500 on A4 to A7, and `zero_effort_controller` supplies the zero torque
overlay throughout. Nothing in this workspace commands a torque during a home move.

`joint_trajectory_controller` and `fri_position_passthrough_controller` claim the same
position command interfaces, so only one of them is active. `move_to_home()` in
`scripts/kuka_crisp.py` runs the trajectory at REST and leaves the arm at REST: the
trajectory controller stays active and holds the last commanded position, which is the
target, or the point where an aborted goal stopped. From ARMED, `crisp_session.py --home`
first goes to REST with one strict switch.

crisp_py's `Robot.home()` is not used on this cell: it moves in a fixed 5 s whatever the
distance, and its switcher sends `BEST_EFFORT`, which reports success after a partial
switch.

### Running it

In T1, enabling switch held, workspace clear, for the whole move:

```bash
pixi run -e jazzy python scripts/crisp_session.py --home
```

It refuses:

- `move_to_home()` when the cell is not at REST. `crisp_session.py --home` goes to REST first,
  by one strict switch, and refuses when neither `joint_trajectory_controller` nor
  `fri_position_passthrough_controller` is active;
- a target angle outside the position limits of the robot description, `[169, 119, 169,
  119, 169, 119, 174]` degrees, so that it is refused here and not by the FRI command
  guard, which ends the session.

The duration is the largest joint delta at 10 deg/s, and at least 2 s: a 165 deg move takes
16.5 s. Override with `--speed`, upward only with care: the FRI command guard trips on
**measured** velocity exceeding the limit of the robot description and answers by stopping
the session.

### Releasing the enabling switch mid-move

It drops `COMMANDING_ACTIVE`, which ends the FRI session and every controller on it, and
needs a relaunch of `pixi run -e jazzy hardware`. Plan the move as one continuous hold.
