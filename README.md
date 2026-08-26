# KUKA LBR iiwa 14 R820 + CRISP with Pixi

Pixi-managed ROS 2 Jazzy workspace that drives a **KUKA LBR iiwa 14 R820** over the
Fast Robot Interface with CRISP joint and Cartesian torque control.

- ROS 2 Jazzy from RoboStack, managed by [Pixi](https://pixi.sh)
- [`lbr_fri_ros2_stack`](https://github.com/lbr-stack/lbr_fri_ros2_stack) for FRI
- [`crisp_controllers`](https://github.com/learnsyslab/crisp_controllers) for torque control
- a small FRI adapter supplying the position half of KUKA's position-plus-torque-overlay command

CRISP torque controllers are deliberately loaded **inactive**. Commission the driver
and state feedback before activating any of them.

## Quick start (no robot needed)

```bash
pixi run -e jazzy setup     # clone sources and build; takes a while the first time
pixi run -e jazzy mock      # start mock hardware
```

Then in a second terminal:

```bash
pixi run -e jazzy controllers
```

That exercises package discovery, URDF names, parameters, and claimed interfaces
without a cabinet. Everything past [Prepare the Sunrise controller](#prepare-the-sunrise-controller)
needs the real robot.

## Task reference

Every task runs under `pixi run -e jazzy <task>`.

| Task | What it does | Moves the arm? |
| --- | --- | --- |
| `setup` | Import sources and build. Idempotent. | no |
| `sources` | Import sources only. | no |
| `build` | Build only (implies `sources`). | no |
| `mock` | Mock hardware bringup, no cabinet required. | no |
| `mock-loopback` | As `mock`, with DDS pinned to loopback for single-host or sandbox use. | no |
| `hardware` | Real FRI bringup. Requires `LBRServer` on the smartPAD. | no, but arms the robot |
| `controllers` | `ros2 control list_controllers` against `/lbr/controller_manager`. | no |
| `interfaces` | `ros2 control list_hardware_interfaces`. | no |
| `session` | Status: controller states, joint positions, EE pose, pre-torque gate. | no |
| `session --home` | **MOVES THE ARM** to the home configuration. | **yes** |
| `session --switch <controller>` | Safe controller switch holding the FRI passthrough active. | no |

Arbitrary commands run in the environment too, for example
`pixi run -e jazzy ros2 topic hz /lbr/joint_states`.

## Select the FRI client version

The FRI client SDK must exactly match the FRI version installed in Sunrise.OS.
LBR-Stack ships manifests for `1.11`, `1.14`, `1.15`, `1.16`, `2.5`, `2.6`, and
`2.7` on ROS 2 Jazzy. It does **not** ship one for `1.17`, so this workspace adds
`repos/repos-fri-1.17.yaml`, which mirrors upstream's 1.16 manifest against the
`fri-1.17` branch of `lbr-stack/fri`. `scripts/import_sources.sh` prefers a local
`repos/repos-fri-<version>.yaml` over the upstream manifest.

The workspace defaults to **FRI 1.17**. If your robot uses another version:

1. pass it while importing sources, for example `FRI_CLIENT_VERSION=1.16`; and
2. set the matching `major_version` and `minor_version` in
   `config/lbr_system_config.yaml` before building.

Only the major version is enforced at runtime; `lbr_ros2_control` compares it
against the compiled SDK and refuses to start on a mismatch. The minor version is
parsed but not checked, so an incorrect minor number fails silently at the FRI
handshake instead.

Do not mix source packages from two FRI versions in the same `src` directory.

## Install and build

Install [Pixi](https://pixi.sh), then:

```bash
FRI_CLIENT_VERSION=1.17 pixi run -e jazzy setup
```

The source task is idempotent. It clones the Jazzy branch of LBR-Stack, imports its
FRI-specific repositories, clones CRISP under `src/`, and clones `crisp_py` under
`external/` for reference.

## Validate without the robot

```bash
pixi run -e jazzy mock
```

For a sandbox or single-host test where DDS should use loopback only, use
`pixi run -e jazzy mock-loopback` instead.

In a second terminal:

```bash
pixi run -e jazzy controllers
pixi run -e jazzy interfaces
```

Expected controller states:

- `fri_position_passthrough_controller`: `active`
- `zero_effort_controller`: `active`
- `pose_broadcaster` and `twist_broadcaster`: `active`
- both CRISP impedance controllers: `inactive`

Mock hardware verifies package discovery, URDF names, parameters, and claimed
interfaces. It does not reproduce KUKA dynamics or validate torque gains.

Jazzy's outer command limiter (`enforce_command_limits`) is disabled in **both** mock
and hardware. It throws `St19bad_optional_access` on the first update cycle and the
controller manager responds by deactivating the offending controller — on hardware
that means losing `fri_position_passthrough_controller` at the moment FRI reaches
`COMMANDING_ACTIVE`, which is the dangerous case. See
`config/hardware_overrides.yaml` for the full reasoning. LBR-Stack's `safe_stop`
command guard is separate and remains authoritative.

## Prepare the Sunrise controller

See [`docs/sunrise_setup.md`](docs/sunrise_setup.md) for the Windows-side procedure,
including two places where LBR-Stack's
[hardware setup guide](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html)
does not apply as written.

The smartPAD settings this workspace expects:

- run the first tests in **T1 mode**;
- FRI send period: **2 ms**;
- FRI control mode: **JOINT_IMPEDANCE_CONTROL**;
- FRI client command mode: **TORQUE**;
- client IP: the ROS workstation's address on the robot-facing subnet.

### Networking is your job

**Find your cabinet's address before anything else** — read it from the smartPAD
network configuration screen or from Workbench's connection dialog. `172.31.1.147`
is the KUKA factory default for X66 (KLI), and `172.31.1.148` is one of the two
client addresses hardcoded in stock `LBRServer.java`, but confirm rather than assume.

Whatever addresses you end up with, the workspace needs:

- the ROS workstation to hold an address on the **same subnet as X66**, on the
  interface cabled to it;
- that address to be one of the two in `client_names_` in `LBRServer.java`, or you
  edit that array;
- **no gateway and no internet route on the cabinet**;
- exactly one default route on the workstation, and not the robot one.

Nothing in this repository configures any of that. See
[Network layout](docs/sunrise_setup.md#network-layout) for the arrangement used here
and [Debugging X66 reachability](docs/sunrise_setup.md#debugging-x66-reachability)
when the cabinet does not answer — note that it does not reply to `ping`, so ARP is
the test, not ICMP.

The ROS side listens on `INADDR_ANY:30200` per `config/lbr_system_config.yaml`, so
it accepts the FRI session regardless of subnet.

Before torque testing, configure the correct tool/load data in Sunrise, clear the
workspace, keep an enabled E-stop within reach, and use reduced T1 speed.

## Working from another machine

The bringup has to run on the workstation cabled to X66 — FRI is a 2 ms UDP loop and
does not tolerate an extra hop. Drive it over SSH:

```bash
ssh <user>@<workstation>
cd path/to/pixi_kuka_ros2
pixi run -e jazzy hardware
```

Bringup, status, and any move each need their own shell, and the bringup must
outlive the SSH connection while you walk to the smartPAD. Use a multiplexer:

```bash
ssh <user>@<workstation>
tmux new -s kuka                       # or: tmux attach -t kuka
pixi run -e jazzy hardware             # leave running; detach with Ctrl-b d
```

```bash
# second window (Ctrl-b c), same host
pixi run -e jazzy session               # status only, safe to run any time
pixi run -e jazzy controllers
```

To subscribe to topics from a *different* machine on the same lab network, match the
DDS settings that `scripts/set_ros_env.sh` exports — `ROS_DOMAIN_ID=42` and
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /lbr/joint_states
```

Do not use `mock-loopback` when you need to reach the workspace from another host;
it pins CycloneDDS to the loopback interface.

## Start hardware in zero-overlay mode

With `LBRServer` waiting for the client:

```bash
pixi run -e jazzy hardware
```

The launch starts:

- the iiwa 14 FRI hardware interface in torque mode;
- `fri_position_passthrough_controller`, mirroring measured joint positions into
  FRI's required position command;
- `zero_effort_controller`, continuously supplying a zero torque overlay;
- state, wrench, pose, and twist broadcasters; and
- the CRISP controllers inactive.

Verify before proceeding:

```bash
pixi run -e jazzy session
pixi run -e jazzy interfaces
pixi run -e jazzy ros2 topic hz /lbr/joint_states
```

Do not continue if positions are not finite, the FRI session is not
`COMMANDING_ACTIVE`, the reported model is not `iiwa14`, or the load-data safety
check fails.

## Move to the home configuration

In T1, with the enabling switch held and the workspace clear:

```bash
pixi run -e jazzy session --home
```

This commands positions, not torques: `joint_trajectory_controller` takes the
position command interfaces for the move while `zero_effort_controller` keeps
supplying the zero torque overlay, and the swap back to
`fri_position_passthrough_controller` happens even if the goal is aborted. Home is
`[0, 30, 0, -75, 0, 75, 0]` degrees, defined in `scripts/kuka_crisp.py`.

Do not use crisp_py's `Robot.home()`. It switches through crisp_py's own switcher,
which deactivates `zero_effort_controller` and sends `BEST_EFFORT`. See
[Moving to the home configuration](docs/commissioning_protocol.md#moving-to-the-home-configuration).

## First CRISP switch

The gains in `config/controllers.yaml` are deliberately conservative starting values,
not validated gains for a particular payload. Keep the position passthrough
controller active and atomically replace the zero-effort controller:

```bash
pixi run -e jazzy session --switch cartesian_impedance_controller
```

Return to zero overlay:

```bash
pixi run -e jazzy session --switch zero_effort_controller
```

Both go through `safe_switch` in `scripts/kuka_crisp.py`, which refuses to switch
unless `fri_position_passthrough_controller` is already active and holds it, plus
`estimated_wrench_interface`, across the switch. The equivalent by hand:

```bash
pixi run -e jazzy ros2 control switch_controllers \
  -c /lbr/controller_manager \
  --deactivate zero_effort_controller \
  --activate cartesian_impedance_controller \
  --strict
```

Never deactivate a CRISP torque controller without activating
`zero_effort_controller` in the same strict switch. FRI retains its last complete
position-plus-torque command when an input becomes invalid, so merely leaving the
effort interface unclaimed is not a safe stop strategy.

## Use the CRISP Python client

The workspace exports `config/` through `CRISP_CONFIG_PATH` and provides an
iiwa-specific client profile. With either mock or hardware bringup running:

```python
from crisp_py.robot import Robot

robot = Robot.from_yaml("iiwa14_r820")
robot.wait_until_ready()

print(robot.joint_values)
print(robot.end_effector_pose)
```

The client publishes in `/lbr` and uses `lbr_link_0` to `lbr_link_ee`, matching the
broadcasters and controllers here.

Do not call the generic `Robot.home()` or crisp_py's unmodified controller switcher
on the physical iiwa. Those paths do not know that KUKA FRI requires
`fri_position_passthrough_controller` to remain active, and they switch with
`BEST_EFFORT`, which reports success after a partial switch. Use
`scripts/kuka_crisp.py`, which wraps both operations:

```python
import sys; sys.path.insert(0, "scripts")
from kuka_crisp import make_kuka_robot, move_to_home, safe_switch

robot = make_kuka_robot()
robot.wait_until_ready()

move_to_home(robot)                                   # positions, zero overlay held
safe_switch(robot, "cartesian_impedance_controller")  # torque overlay
```

## Architecture note

CRISP's Cartesian controller claims `effort` command interfaces. KUKA FRI's torque
mode requires both joint positions and torque overlays. The two local adapter
controllers make that mismatch explicit:

```text
measured positions -> FRIPositionPassthrough -> joint position commands
CRISP controller   -> effort interfaces      -> torque overlays
                                          OR
ZeroEffort         -> effort interfaces      -> zero overlays
```

Only one of `ZeroEffort` and a CRISP controller may own the effort interfaces at a
time. The position passthrough controller remains active in both cases.

## Documentation

- [`docs/sunrise_setup.md`](docs/sunrise_setup.md) — Windows-side `LBRServer`
  installation, network requirements, and X66 reachability debugging.
- [`docs/commissioning_protocol.md`](docs/commissioning_protocol.md) — startup
  procedure, position referencing, operating modes and safety configuration, the
  pre-FRI gate, and recorded failure modes with their recoveries.

## License

Apache-2.0. See [`LICENSE`](LICENSE).
