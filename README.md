# pixi_kuka_ros2

The ROS 2 driver stack of a KUKA LBR iiwa 14 R820, in one pixi environment (Jazzy from RoboStack): the arm
over KUKA's Fast Robot Interface with [`lbr_fri_ros2_stack`](https://github.com/lbr-stack/lbr_fri_ros2_stack),
torque control with [`crisp_controllers`](https://github.com/learnsyslab/crisp_controllers), and two
controllers of its own that join them.

This revision has not run on the arm; see "Checked".

FRI's torque mode takes a joint position and a torque overlay every cycle, and Sunrise runs its joint spring
(200 Nm/rad in `LBRServer`) around that position. crisp writes only torques. So this repo adds
`fri_position_passthrough_controller`, which writes the position, and `zero_effort_controller`, which writes
a zero overlay when no crisp controller runs.

The stack is in one of two states:

| State | Active | What holds the arm |
| --- | --- | --- |
| REST | `joint_trajectory_controller`, `zero_effort_controller` | Sunrise's joint spring, on the fixed position the trajectory controller commands |
| ARMED | `fri_position_passthrough_controller`, `cartesian_impedance_controller` | The torque controller. The passthrough's position follows the arm, so Sunrise's spring adds nothing |

It comes up at REST. One strict switch of all four controllers goes from one state to the other.
`joint_impedance_controller` can be armed in place of the Cartesian one. The passthrough with
`zero_effort_controller` holds nothing: the arm sags, and with a tool that is not declared to Sunrise it
falls.

## What you need

- A Linux x86-64 PC with [pixi](https://pixi.sh), cabled to the cabinet's X66 (KLI) port, with the address
  `172.31.1.148` on that interface. The stack runs on this PC: FRI is a 2 ms UDP loop and takes no extra
  network hop.
- Sunrise.OS with FRI 1.17 on the cabinet, and the `LBRServer` application on it
  ([`docs/sunrise_setup.md`](docs/sunrise_setup.md)). The client SDK built here is FRI 1.17, and
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
| `mock-offset` | The mock stack with a position offset, for `check-mock --offset-lagged`. | no: domain 77 on loopback |
| `check-mock` | The bench check of the passthrough's modes, against a mock that is up. | no: it moves the mock, and refuses anything else |
| `controllers`, `interfaces` | `ros2 control list_controllers` and `list_hardware_interfaces`. | no |
| `session` | `scripts/crisp_session.py` on domain 77 on loopback: it reaches the mock only. | no |
| `position-source` | `scripts/position_source.py`: what the passthrough writes. | no. `set` changes what holds the arm from the next arm on |
| `position-source-mock` | The same on domain 77 on loopback. | no |

`scripts/set_ros_env.sh` sets the environment of every task: CycloneDDS, and `ROS_DOMAIN_ID` 42 unless
it is set already. The tasks that say domain 77 set it themselves, with
`config/cyclonedds_loopback.xml`. `controllers`, `interfaces`, `position-source` and anything run as
`pixi run -e jazzy python ...` are on the shell's domain: on the cell's PC they reach the arm's stack.
Another machine on the lab network reads the topics with the same domain and CycloneDDS.

Two scripts move the real arm. They have no task and run as `pixi run -e jazzy python scripts/<name>`:

- `crisp_session.py` prints the controllers, the joints and the flange pose. `--home` moves the arm to
  the home joints with the trajectory controller and leaves it at REST there. `--home-file F` takes the
  joints from a JSON file. `--switch NAME` arms a torque controller from REST, or goes back to REST with
  `zero_effort_controller`.
- `track_tune.py replay REC --gains A.yaml B.yaml --restore A.yaml --out DIR` sends a recorded target
  trajectory to the armed arm once per gains file and scores how the arm followed. It asks an operator at a
  terminal for `go`.

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
   attached in `LBRServer`. Declare the load; do not raise the limit.
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
   manager does not activate again what it deactivated. `crisp_session.py` then shows
   `cell: neither REST nor ARMED` and refuses to switch. Back to REST by hand:

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
| `/lbr/target_pose` | `PoseStamped`, in | the sender's | The target of `cartesian_impedance_controller`. With two publishers on it, crisp and the passthrough ignore it. |
| `/lbr/target_joint` | `JointState`, in | the sender's | The joint target of the crisp controllers. |
| `/lbr/fri_position_passthrough_controller/state` | `Float64MultiArray` | 500 Hz while it is active | Its mode, whether its target is stale, and per joint what it read, solved and wrote. [`docs/position_source.md`](docs/position_source.md#state) |
| `/lbr/kuka_external_torque` | `JointState` | one per `/lbr/lbr_state` message, `hardware` only | Effort is KUKA's external torque estimate in Nm. From `scripts/kuka_ext_torque_relay.py`. |
| `/lbr/force_torque_broadcaster/wrench` | `WrenchStamped` | `hardware` only | The wrench estimated from the joint torques, at `lbr_link_ee`. |
| `/lbr/controller_manager/introspection_data/values` | `StatisticsValues` | 500 Hz | The terms of the active crisp controller's law. The names are on `.../names`. |
| `/lbr/robot_description` | `String`, latched | once | The URDF the stack runs. |
| `/lbr/controller_manager/list_controllers`, `switch_controller` | services | | |
| `/lbr/joint_trajectory_controller/follow_joint_trajectory` | action | | Joint moves at REST. |

What the passthrough writes into the position field is its parameter `position_source`: the measured
position (`measured`, the default), a low-pass of it (`lagged`) or the IK of `/lbr/target_pose`
(`ik_target`). [`docs/position_source.md`](docs/position_source.md)

## The UMI: `hardware-umi`

`hardware-umi` launches the same stack with `tool:=umi`. The description is `iiwa14_umi.urdf.xacro` of
`iris_robots_description` (revision `93c2d99`, pinned in `scripts/import_sources.sh`), and
`config/controllers_umi.yaml` goes on top of `config/controllers.yaml`:

- `/lbr/current_pose`, `/lbr/current_twist` and `/lbr/target_pose` are of the wrist camera,
  `lbr_umi_camera`, not of the flange. Nothing that speaks `lbr_link_ee` may command `/lbr/target_pose`
  then: it would put the camera where it meant the flange, about 90 mm and 25 deg away.
- The wrench is estimated at `lbr_umi_tweezer_tip`, and its force thresholds are zero.
- The Cartesian gains are 4000 N/m and 250 Nm/rad, and `limit_error` is off, so `error_clip` does not bound
  the task force.

The camera frame is the CAD value of the description, not a hand-eye measurement.

## Checked

On lev-workstation, 2026-11-24, Jazzy, ROS domain 77 on loopback, mock hardware. No cabinet is on that
machine.

- **The position source law:** the 20 gtests of `test-controllers` pass.
- **Bringup:** `mock-loopback` comes up at REST: `joint_trajectory_controller`, `zero_effort_controller` and
  the broadcasters active, the passthrough and the two crisp controllers inactive. `/lbr/joint_states` and the
  passthrough's state at 500 Hz, `/lbr/current_pose` and `/lbr/current_twist` at 250 Hz.
- **The passthrough's modes:** `check-mock` passes 72 of 72 checks, and `check-mock --offset-lagged` on
  `mock-offset` 19 of 19. They cover `measured` bit for bit, the time constant of `lagged`, the IK of
  `ik_target` against an IK computed in the check, both clamps, the stale fallback, two publishers, and what
  `position_source.py` refuses.
- **Session:** `crisp_session.py` prints the state; `--home --home-file` brings the mock arm to the file's
  joints within 2e-6 deg; `--switch` goes REST, ARMED, REST with one strict switch each way. Refused, each
  with exit 1: a controller that is no torque controller, a home target outside the description's limits,
  `joint_impedance_controller` over a passthrough that is set to or runs `lagged`, a move or a switch with
  no owner of the position command, a move without `zero_effort_controller`.
- **Replay:** `track_tune.py replay` of one recorded 20 s target trajectory under two gains files, on the
  mock in `ik_target`, where the mock arm follows the target: both runs complete and score. It refuses to
  start at REST, with a second publisher, without a terminal, and without `go`.

Not checked:

- Anything on the arm, for this revision. That includes the passthrough holding the last finite position
  of a joint whose state is not finite, which no bench check reaches.
- `lagged` and `ik_target` have not been checked on the arm.
- The two Sunrise applications in `sunrise/` are not compiled here: there is no Sunrise SDK on Linux.
- The gains of `joint_impedance_controller` (100, 100, 75, 75, 40, 25, 15 Nm/rad, half of Sunrise's
  200 Nm/rad on A1 and A2) were never checked on the arm, in particular not for what two springs in series
  do.
- `hardware-umi`, and the hardware-only parts of the launch: the FRI broadcasters, the wrench estimate and
  the torque relay.
