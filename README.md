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
