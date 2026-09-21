# KUKA LBR iiwa 14 R820 + CRISP with Pixi

Pixi-managed ROS 2 Jazzy workspace that drives a **KUKA LBR iiwa 14 R820** over the
Fast Robot Interface with CRISP joint and Cartesian torque control.

- ROS 2 Jazzy from RoboStack, managed by [Pixi](https://pixi.sh)
- [`lbr_fri_ros2_stack`](https://github.com/lbr-stack/lbr_fri_ros2_stack) for FRI
- [`crisp_controllers`](https://github.com/learnsyslab/crisp_controllers) for torque control
- a small FRI adapter supplying the position half of KUKA's position-plus-torque-overlay command

> CRISP torque controllers load **inactive** by design. Commission the driver and
> state feedback before activating any of them. Every step that moves the arm belongs
> in T1 with the enabling switch held.

## Quick start (no robot needed)

```bash
pixi run -e jazzy setup     # clone sources and build; slow the first time
pixi run -e jazzy mock      # mock hardware bringup
pixi run -e jazzy session   # second terminal: controller states and joint positions
```

That exercises package discovery, URDF names, parameters, and claimed interfaces
without a cabinet. It does not reproduce KUKA dynamics or validate torque gains.

## Tasks

All run as `pixi run -e jazzy <task>`. Arbitrary commands work too, for example
`pixi run -e jazzy ros2 topic hz /lbr/joint_states`.

| Task | What it does | Moves the arm? |
| --- | --- | --- |
| `setup` | Import sources and build. Idempotent. | no |
| `sources` / `build` | The two halves of `setup`. | no |
| `mock` | Mock hardware bringup, no cabinet required. | no |
| `mock-loopback` | As `mock`, DDS pinned to loopback for single-host use. | no |
| `hardware` | Real FRI bringup. Requires `LBRServer` on the smartPAD. | no, but arms the robot |
| `hardware-umi` | As `hardware`, with the UMI in the description and the Cartesian endpoint at `lbr_umi_camera`. | no, but arms the robot |
| `mock-umi` | Mock bringup of the same, on loopback. | no |
| `controllers` / `interfaces` | List controllers / hardware interfaces. | no |
| `session` | Controller states, joint positions, EE pose, pre-torque gate. | no |
| `session --home` | **MOVES THE ARM** to the home configuration. | **yes** |
| `session --switch <name>` | Safe switch, holding the FRI passthrough active. | no |

## Bringing up the real robot

Work through these in order. Each links to the detail.

**1. Match the FRI version.** The client SDK must exactly match the FRI version in
Sunrise.OS. This workspace defaults to 1.17 and adds a manifest for it, since
LBR-Stack does not ship one on Jazzy.
→ [`docs/fri_version.md`](docs/fri_version.md)

**2. Build.**

```bash
FRI_CLIENT_VERSION=1.17 pixi run -e jazzy setup
```

**3. Set up the network.** Nothing here configures it for you. Find your cabinet's
address on the smartPAD, put the workstation on the same subnet on the interface
cabled to X66, and give the cabinet no gateway. The cabinet does not answer `ping`,
so ARP is the reachability test.
→ [`docs/sunrise_setup.md`](docs/sunrise_setup.md#network-layout)

**4. Install `LBRServer` on the cabinet.** Windows-side, through Sunrise.Workbench,
including two places where LBR-Stack's upstream guide does not apply as written.
→ [`docs/sunrise_setup.md`](docs/sunrise_setup.md)

**5. Clear the commissioning gate.** Position referencing, safety configuration,
operating mode, tool and load data. Do not start FRI until every item passes.
→ [`docs/commissioning_protocol.md`](docs/commissioning_protocol.md#gate-before-fri)

**6. Start in zero-overlay mode.** With `LBRServer` waiting for the client:

```bash
pixi run -e jazzy hardware
pixi run -e jazzy session      # verify before doing anything else
```

Stop if positions are not finite, the FRI session is not `COMMANDING_ACTIVE`, the
model is not `iiwa14`, or the load-data check fails.

**7. Move to home.** In T1, enabling switch held, workspace clear:

```bash
pixi run -e jazzy session --home
```

Commands positions, not torques, and returns the position command to the passthrough
even if the goal aborts.
→ [`docs/commissioning_protocol.md`](docs/commissioning_protocol.md#moving-to-the-home-configuration)

**8. Activate a torque controller.**

```bash
pixi run -e jazzy session --switch cartesian_impedance_controller
pixi run -e jazzy session --switch zero_effort_controller
```

Never deactivate a CRISP torque controller without activating `zero_effort_controller`
in the same strict switch — FRI keeps its last complete command, so an unclaimed
effort interface is not a stop.
→ [`docs/controllers.md`](docs/controllers.md)

## How it fits together

CRISP claims `effort` command interfaces. FRI's torque mode needs positions *and*
torque overlays every cycle:

```text
measured positions -> FRIPositionPassthrough -> joint position commands
CRISP controller   -> effort interfaces      -> torque overlays
                                          OR
ZeroEffort         -> effort interfaces      -> zero overlays
```

Exactly one of `ZeroEffort` and a CRISP controller owns the effort interfaces at a
time; the position passthrough stays active for both.
→ [`docs/controllers.md`](docs/controllers.md)

## Documentation

| Document | Covers |
| --- | --- |
| [`fri_version.md`](docs/fri_version.md) | Choosing and pinning the FRI client SDK version |
| [`sunrise_setup.md`](docs/sunrise_setup.md) | Windows-side `LBRServer` install, network requirements, X66 reachability debugging |
| [`commissioning_protocol.md`](docs/commissioning_protocol.md) | Startup procedure, position referencing, operating modes and safety configuration, the pre-FRI gate, recorded failure modes |
| [`controllers.md`](docs/controllers.md) | Controller architecture, safe switching, the CRISP Python client |
| [`remote_access.md`](docs/remote_access.md) | Running the bringup over SSH |

## License

Apache-2.0. See [`LICENSE`](LICENSE).

## The UMI, and driving the camera

With the UMI bolted to the flange, `pixi run -e jazzy hardware-umi` launches the same stack with
`iris_robots_description`'s `iiwa14_umi.urdf.xacro` (pinned in `scripts/import_sources.sh`;
tweezer jaws, mount clock 135 deg) and `config/controllers_umi.yaml` on top, which moves
`pose_broadcaster`, `twist_broadcaster` and `cartesian_impedance_controller` to
`lbr_umi_camera`. `/lbr/current_pose` is then the camera pose, and `/lbr/target_pose` is a
camera target — the frame a hand-held-trained policy acts in.

**Nothing that speaks `lbr_link_ee` may run against it**: iiwa-next's excitation player,
`haply_teleop_bridge.py`, `crisp_session.py`'s pose readout. They would put the camera where
they meant the flange — ~90 mm and 25 deg away. Excitation and teleop stay on `hardware`.

```bash
pixi run -e jazzy hardware-umi
pixi run -e jazzy python scripts/camera_gizmo.py     # prints a LAN link per address
```

`camera_gizmo.py` serves a viser page with the arm, the measured camera (with a frustum along
its optical axis) and a transform gizmo on it. **arm** raises the Cartesian controller from
rest in one STRICT switch, **engaged** starts publishing, and dragging the gizmo — or the
camera-axis nudge buttons — moves the camera. The target walks at the speed sliders (0.05 m/s,
15 deg/s to start), stays within 30 mm / 10 deg of the measured pose, and inside a box around
the camera's home position. It refuses a stack whose `robot_description` has no camera frame
or whose controllers are on another frame, and disengages if `/lbr/current_pose` stops
agreeing with its own FK of `lbr_umi_camera`. `--dry-run` does everything but publish.

Under **correction**, two client-side fixes for the arm trailing the target, both off by
default: an integral for the static offset (the undeclared UMI's sag and stiction), and a
velocity lead of (D/K)·v, read off the live controller's gains, for the damping drag while
moving. Compare them with **gizmo - arm, 2 s**.

