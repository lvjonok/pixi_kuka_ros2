# pixi_kuka_ros2

The ROS 2 driver stack of a KUKA LBR iiwa 14 R820, in one pixi environment (Jazzy from RoboStack): the arm
over KUKA's Fast Robot Interface with [`lbr_fri_ros2_stack`](https://github.com/lbr-stack/lbr_fri_ros2_stack),
torque control with [`crisp_controllers`](https://github.com/learnsyslab/crisp_controllers), and two
controllers of its own that join them.

This revision has not run on the arm; see "Checked".

FRI's torque mode takes a joint position and a torque overlay every cycle, and Sunrise runs its joint spring
around that position. crisp writes only torques. So this repo adds `fri_position_passthrough_controller`,
which writes the position, and `zero_effort_controller`, which writes a zero overlay when no crisp controller
runs. The position the passthrough writes is the IK of crisp's Cartesian target, held within a small gap of
the measured position: Sunrise's spring pulls toward the target too, with a bounded torque.

The stack is in one of two states:

| State | Active | What holds the arm |
| --- | --- | --- |
| REST | `joint_trajectory_controller`, `zero_effort_controller` | Sunrise's joint spring, on the fixed position the trajectory controller commands |
| ARMED | `fri_position_passthrough_controller`, `cartesian_impedance_controller` | The torque controller together with Sunrise's joint spring, which the passthrough centres on the IK of the target |

It comes up at REST. One strict switch of all four controllers goes from one state to the other. The
passthrough with `zero_effort_controller` does not hold the arm: the arm sags, and with a tool that is not
declared to Sunrise it falls.

`joint_impedance_controller` is loaded and stays inactive. `crisp_session.py` does not arm it: Sunrise's
spring pulls toward the IK of `/lbr/target_pose`, which only `cartesian_impedance_controller` follows.

## What you need

- A Linux x86-64 PC with [pixi](https://pixi.sh), cabled to the cabinet's X66 (KLI) port, with the address
  `172.31.1.148` on that interface. The stack runs on this PC: FRI is a 2 ms UDP loop and takes no extra
  network hop.
- Sunrise.OS with FRI 1.17 on the cabinet, and the `LBRServer` application on it
  ([`docs/sunrise_setup.md`](docs/sunrise_setup.md)), or another torque overlay application: see
  "The Sunrise application and the spring". The client SDK built here is FRI 1.17, and
  `config/lbr_system_config.yaml` names that version. The driver refuses to start on another major
  version. It does not compare the minor version: a wrong one fails at the FRI handshake without a clear
  error.
- For `hardware-umi`: read access to the lab's private `iris_robots_description`.

## Build

```bash
pixi run -e jazzy setup
```

It clones the sources at the revisions in `repos/repos-fri-1.17.yaml` into `src/`, puts the four patches of
[`patches/`](patches/README.md) on `crisp_controllers`, clones `iris_robots_description` if it can, and
builds with colcon. Without `iris_robots_description` the build goes on and only `hardware-umi` cannot run.

## Tasks

All as `pixi run -e jazzy <task>`.

| Task | What it does | Can it move the arm? |
| --- | --- | --- |
| `setup` | `sources`, then `build`. | no |
| `sources` | Clone and patch the sources. | no |
| `build` | Build the workspace. | no |
| `test-controllers` | Build this package with its tests and run them: the gtests of the position source law. | no |
| `hardware` | The stack on the cabinet. Needs `LBRServer` started on the smartPAD. | It opens the FRI session and holds the arm at REST |
| `hardware-umi` | `hardware` with the UMI in the description. See below. | as `hardware` |
| `mock-loopback` | The stack on mock hardware, no cabinet. | no: domain 77 on loopback |
| `check-mock` | The bench check of the passthrough, against a mock that is up. | no: it moves the mock, and refuses anything else |
| `controllers`, `interfaces` | `ros2 control list_controllers` and `list_hardware_interfaces`. | no |
| `session` | `scripts/crisp_session.py` on domain 77 on loopback: it reaches the mock only. | no |

`scripts/set_ros_env.sh` sets the environment of every task: CycloneDDS, and `ROS_DOMAIN_ID` 42 unless
it is set already. The tasks that say domain 77 set it themselves, with
`config/cyclonedds_loopback.xml`. `controllers`, `interfaces` and anything run as
`pixi run -e jazzy python ...` or `pixi run -e jazzy ros2 ...` are on the shell's domain: on the cell's PC
they reach the arm's stack. Another machine on the lab network reads the topics with the same domain and
CycloneDDS.

Two scripts move the real arm. They have no task and run as `pixi run -e jazzy python scripts/<name>`:

- `crisp_session.py` prints the controllers, the joints and the flange pose. `--home` moves the arm to
  the home joints with the trajectory controller and leaves it at REST there. `--home-file F` takes the
  joints from a JSON file. `--switch cartesian_impedance_controller` arms from REST, and
  `--switch zero_effort_controller` goes back to REST.
- `track_tune.py replay REC --gains A.yaml B.yaml --restore A.yaml --out DIR` sends a recorded target
  trajectory to the armed arm once per gains file and scores how the arm followed. It asks an operator at a
  terminal for `go`.

## The Sunrise application and the spring

The stiffness of Sunrise's joint spring is set by the application that runs on the cabinet, and FRI does not
report it. The passthrough takes it from its parameter `sunrise_joint_stiffness` and keeps its position
within `max_spring_torque_nm / sunrise_joint_stiffness` of the measured one, so that the spring pulls with
at most `max_spring_torque_nm` (5, 5, 5, 4, 2.5, 2.5, 1.5 Nm on A1 to A7).

**`sunrise_joint_stiffness` must be the stiffness of the application that runs, and never below it.** Its
value in `config/controllers.yaml`, 1000 Nm/rad on A1 to A3 and 500 on A4 to A7, is that of
[`sunrise/FRITorqueDriver.java`](sunrise/FRITorqueDriver.java). Under `LBRServer`, which runs 200 Nm/rad on
every joint, the same parameters give a weaker spring, a fifth of the cap on A1 to A3 and two fifths on A4
to A7: the safe side. A parameter below the application's stiffness is the unsafe side: the spring then
exceeds the cap by the ratio of the two.
[`docs/position_source.md`](docs/position_source.md#the-sunrise-application-and-sunrise_joint_stiffness)

## Bringing up the real arm

In this order.

1. Put the PC on the cabinet's subnet and check that X66 answers ARP. The cabinet does not answer ping.
   [`docs/sunrise_setup.md`](docs/sunrise_setup.md#network-layout)
2. Install `LBRServer` on the cabinet. [`docs/sunrise_setup.md`](docs/sunrise_setup.md#procedure)
3. Go through the list before FRI: referencing, safety configuration, operating mode, tool and load data.
   [`docs/commissioning_protocol.md`](docs/commissioning_protocol.md#before-fri-starts)
4. Start the stack on the PC, in a tmux session so that it outlives the shell while you are at the smartPAD:

   ```bash
   pixi run -e jazzy hardware
   ```

   It waits up to 600 s for the cabinet. Then start `LBRServer` on the smartPAD and answer its dialogs:
   send period `2` ms, remote address `172.31.1.148`, `JOINT_IMPEDANCE_CONTROL`, `TORQUE`. The stack comes
   up at REST. The driver checks the estimated external torque once, when FRI starts commanding, against
   2 Nm (`config/lbr_system_config.yaml`): a tool passes only with its load declared in Sunrise and
   attached in `LBRServer`. Declare the load; do not raise the limit. Whatever application runs,
   `sunrise_joint_stiffness` must not be below its stiffness.
5. Look at it:

   ```bash
   pixi run -e jazzy controllers
   pixi run -e jazzy python scripts/crisp_session.py
   ```

6. Move to the home joints, `[0, 30, 0, -75, 0, 75, 0]` degrees. **This moves the arm**: T1, enabling
   switch held for the whole move, workspace clear.

   ```bash
   pixi run -e jazzy python scripts/crisp_session.py --home
   ```

   [`docs/commissioning_protocol.md`](docs/commissioning_protocol.md#moving-to-the-home-configuration)
7. Arm, and go back to REST:

   ```bash
   pixi run -e jazzy python scripts/crisp_session.py --switch cartesian_impedance_controller
   pixi run -e jazzy python scripts/crisp_session.py --switch zero_effort_controller
   ```

   Arm from near the home. The passthrough's IK starts where the arm is, but far from the home its joints
   drift away from the arm's while it solves the target, and the consumer's arming check has refused an
   arming for that. Until a target arrives on `/lbr/target_pose` the passthrough writes the measured
   position. It goes back to it when the newest target is older than `target_timeout_s` (0.1 s) or the
   topic has a second publisher, until a good target arrives. A target that is not finite is dropped.

   Each is one strict switch. By hand they are:

   ```bash
   pixi run -e jazzy ros2 control switch_controllers -c /lbr/controller_manager --strict \
     --activate fri_position_passthrough_controller cartesian_impedance_controller \
     --deactivate joint_trajectory_controller zero_effort_controller
   pixi run -e jazzy ros2 control switch_controllers -c /lbr/controller_manager --strict \
     --activate joint_trajectory_controller zero_effort_controller \
     --deactivate fri_position_passthrough_controller cartesian_impedance_controller
   ```

   Never deactivate a torque controller without activating `zero_effort_controller` in the same strict
   switch. FRI keeps the last complete command, so an effort interface nobody writes is not a stop.

   A strict switch that fails while a controller activates leaves all four inactive: the controller
   manager does not activate again what it deactivated. The passthrough fails its activation on a
   parameter it refuses, a `position_source` other than `ik_target` among them.
   `crisp_session.py` then shows `cell: neither REST nor ARMED` and refuses to switch. Back to REST by
   hand:

   ```bash
   pixi run -e jazzy ros2 control switch_controllers -c /lbr/controller_manager --strict \
     --activate joint_trajectory_controller zero_effort_controller
   ```

Releasing the enabling switch ends the FRI session and every controller on it. The stack is then
started again from step 4.

From Python, `scripts/kuka_crisp.py` gives a crisp_py `Robot` for this cell (`make_kuka_robot`), the switch
(`safe_switch`) and the joint move (`move_to_home`). crisp_py's own `Robot.home()` and controller switcher
are not for this cell: the switcher deactivates every controller that is not a broadcaster, with
`BEST_EFFORT`, and `home()` moves in a fixed 5 s.

## On the wire

Everything is under `/lbr`. The control loop runs at 500 Hz.

| Name | Type | Rate | Holds |
| --- | --- | --- | --- |
| `/lbr/joint_states` | `JointState` | 500 Hz | Position, velocity and effort of `lbr_A1` to `lbr_A7`. |
| `/lbr/current_pose` | `PoseStamped` | 250 Hz | `lbr_link_ee` in `lbr_link_0`. |
| `/lbr/current_twist` | `TwistStamped` | 250 Hz | The twist of the same frame. |
| `/lbr/target_pose` | `PoseStamped`, in | the sender's | The target of `cartesian_impedance_controller` and of the passthrough's IK. With two publishers on it, both ignore it. |
| `/lbr/target_joint` | `JointState`, in | the sender's | The joint target of the crisp controllers. |
| `/lbr/fri_position_passthrough_controller/state` | `Float64MultiArray` | 500 Hz while it is active | A constant 2, whether its target is stale, and per joint what it read, solved and wrote. [`docs/position_source.md`](docs/position_source.md#state) |
| `/lbr/kuka_external_torque` | `JointState` | one per `/lbr/lbr_state` message, `hardware` only | Effort is KUKA's external torque estimate in Nm. From `scripts/kuka_ext_torque_relay.py`. |
| `/lbr/force_torque_broadcaster/wrench` | `WrenchStamped` | `hardware` only | The wrench estimated from the joint torques, at `lbr_link_ee`. |
| `/lbr/controller_manager/introspection_data/values` | `StatisticsValues` | 500 Hz | The terms of the active crisp controller's law. The names are on `.../names`. |
| `/lbr/robot_description` | `String`, latched | once | The URDF the stack runs. |
| `/lbr/controller_manager/list_controllers`, `switch_controller` | services | | |
| `/lbr/joint_trajectory_controller/follow_joint_trajectory` | action | | Joint moves at REST. |

The passthrough writes the IK of `/lbr/target_pose` into the position field, and the measured position
while it has no fresh target. Its parameter `position_source` has the one value `ik_target`.
[`docs/position_source.md`](docs/position_source.md)

## The UMI: `hardware-umi`

`hardware-umi` launches the same stack with `tool:=umi`. The description is `iiwa14_umi.urdf.xacro` of
`iris_robots_description` (revision `93c2d99`, pinned in `scripts/import_sources.sh`), and
`config/controllers_umi.yaml` goes on top of `config/controllers.yaml`:

- `/lbr/current_pose`, `/lbr/current_twist` and `/lbr/target_pose` are of the wrist camera,
  `lbr_umi_camera`, not of the flange, and the passthrough's IK solves for that frame. Nothing that speaks
  `lbr_link_ee` may command `/lbr/target_pose` then: it would put the camera where it meant the flange,
  about 90 mm and 25 deg away.
- The wrench is estimated at `lbr_umi_tweezer_tip`, and its force thresholds are zero.
- The Cartesian gains are 4000 N/m and 250 Nm/rad, and `limit_error` is off, so `error_clip` does not bound
  the task force.

The camera frame is the CAD value of the description, not a hand-eye measurement.

## Checked

On lev-workstation, 2026-11-24, Jazzy, ROS domain 77 on loopback, mock hardware. No cabinet is on that
machine.

- **The position source law:** the 16 gtests of `test-controllers` pass.
- **Bringup:** `mock-loopback` comes up at REST: `joint_trajectory_controller`, `zero_effort_controller` and
  the broadcasters active, the passthrough and the two crisp controllers inactive. `/lbr/joint_states` and the
  passthrough's state at 500 Hz, `/lbr/current_pose` and `/lbr/current_twist` at 250 Hz. The passthrough's
  parameters are those of `config/controllers.yaml`.
- **The passthrough:** `check-mock` passes 47 of 47 checks. They cover arming and resting, the IK against
  an IK computed in the check, both clamps, the stale fallback, two publishers and the first value of the
  state.
- **A refused activation:** with `position_source` set to another value, arming fails and leaves all four
  controllers inactive, and the strict switch by hand brings REST back.
- **Session:** `crisp_session.py` prints the state; `--home --home-file` brings the mock arm to the file's
  joints within 2e-6 deg; `--switch` goes REST, ARMED, REST with one strict switch each way, and while
  ARMED the mock arm reaches a target 20 mm from its pose. Refused, each with exit 1:
  `joint_impedance_controller` from REST and from ARMED, a controller that is no torque controller, a home
  target outside the description's limits, a move or a switch with no owner of the position command, a
  move without `zero_effort_controller`.
- **Replay:** `track_tune.py replay` of one recorded 20 s target trajectory under two gains files, on the
  mock, where the mock arm follows the target: both runs complete and score. It refuses to start at REST,
  with a second publisher, without a terminal, and without `go`.

Not checked:

- This revision has not run on the arm. That includes the passthrough holding the last finite position
  of a joint whose state is not finite, which no bench check reaches.
- The IK of the target as the position command ran on the arm at the older revision the cell runs, where
  it was a setting and not the default. How a teleoperated target behaves under it is not known: the
  cell's teleoperation sessions ran with the measured position as the position command.
- The two Sunrise applications in `sunrise/` are not compiled here: there is no Sunrise SDK on Linux.
- The gains of `joint_impedance_controller` (100, 100, 75, 75, 40, 25, 15 Nm/rad, half of Sunrise's
  200 Nm/rad on A1 and A2) were never checked on the arm, in particular not for what two springs in series
  do.
- `hardware-umi`, and the hardware-only parts of the launch: the FRI broadcasters, the wrench estimate and
  the torque relay.
