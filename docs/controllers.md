# Controllers, switching, and the Python client

How this workspace bridges CRISP and KUKA FRI, and how to drive it safely.

## Why there are two adapter controllers

CRISP's Cartesian controller claims `effort` command interfaces. KUKA FRI's torque
mode requires **both** joint positions and torque overlays on every cycle. The two
local adapter controllers make that mismatch explicit:

```text
measured positions -> FRIPositionPassthrough -> joint position commands
CRISP controller   -> effort interfaces      -> torque overlays
                                          OR
ZeroEffort         -> effort interfaces      -> zero overlays
```

`fri_position_passthrough_controller` mirrors measured joint positions into FRI's
required position command, pinning the position setpoint to where the arm actually is
while CRISP writes the torque overlay on top.

Only one of `ZeroEffort` and a CRISP controller may own the effort interfaces at a
time. The position passthrough stays active in both cases.

## What comes up, and in what state

`pixi run -e jazzy hardware` starts:

- the iiwa 14 FRI hardware interface in torque mode;
- `fri_position_passthrough_controller` — **active**;
- `zero_effort_controller` — **active**, continuously supplying a zero torque overlay;
- `joint_state_broadcaster`, `lbr_state_broadcaster`, `estimated_wrench_interface`,
  `force_torque_broadcaster`, `pose_broadcaster`, `twist_broadcaster` — **active**;
- `cartesian_impedance_controller`, `joint_impedance_controller`,
  `joint_trajectory_controller` — **inactive**.

`joint_trajectory_controller` is inactive because it claims the same position command
interfaces as the passthrough. Activating it is a deliberate strict swap; see
[Moving to the home configuration](commissioning_protocol.md#moving-to-the-home-configuration).

## Switching to a torque controller

```bash
pixi run -e jazzy session --switch cartesian_impedance_controller
pixi run -e jazzy session --switch zero_effort_controller
```

Both go through `safe_switch` in `scripts/kuka_crisp.py`, which refuses to switch
unless `fri_position_passthrough_controller` is already active, and holds it plus
`estimated_wrench_interface` across the switch.

The equivalent by hand:

```bash
pixi run -e jazzy ros2 control switch_controllers \
  -c /lbr/controller_manager \
  --deactivate zero_effort_controller \
  --activate cartesian_impedance_controller \
  --strict
```

**Never deactivate a CRISP torque controller without activating
`zero_effort_controller` in the same strict switch.** FRI retains its last complete
position-plus-torque command when an input becomes invalid, so leaving the effort
interface unclaimed is not a safe stop — the arm keeps the last overlay it was given.

The gains in `config/controllers.yaml` are deliberately conservative starting values,
not validated gains for any particular payload.

## Command limits are disabled on purpose

Jazzy's outer command limiter (`enforce_command_limits`) is disabled in **both** mock
and hardware. It throws `St19bad_optional_access` on the first update cycle, when a
command interface optional is still empty, and the controller manager responds by
deactivating the offending controller:

```text
Caught exception of type : St19bad_optional_access while updating
controller 'fri_position_passthrough_controller': bad optional access
Deactivating controllers : [ fri_position_passthrough_controller ]
```

On hardware that means losing the position passthrough at the instant FRI reaches
`COMMANDING_ACTIVE`, which is the dangerous case. See `config/hardware_overrides.yaml`
for the full record. LBR-Stack's `safe_stop` command guard is a separate mechanism and
remains authoritative.

## The CRISP Python client

The workspace exports `config/` through `CRISP_CONFIG_PATH` and provides an
iiwa-specific client profile in `config/robots/iiwa14_r820.yaml`. With either mock or
hardware bringup running:

```python
from crisp_py.robot import Robot

robot = Robot.from_yaml("iiwa14_r820")
robot.wait_until_ready()

print(robot.joint_values)
print(robot.end_effector_pose)
```

The client publishes in `/lbr` and uses `lbr_link_0` to `lbr_link_ee`, matching the
broadcasters and controllers here.

### Do not use the stock home or switcher

`Robot.home()` and crisp_py's unmodified `ControllerSwitcherClient` are both unsafe on
this robot:

- the switcher deactivates every active controller not ending in `broadcaster` unless
  it is explicitly held, which would drop `fri_position_passthrough_controller`;
- it issues every switch with `BEST_EFFORT`, which reports success after a *partial*
  switch.

Use the wrappers in `scripts/kuka_crisp.py` instead, which send `STRICT` and hold the
passthrough and wrench interface active:

```python
import sys; sys.path.insert(0, "scripts")
from kuka_crisp import make_kuka_robot, move_to_home, safe_switch

robot = make_kuka_robot()
robot.wait_until_ready()

move_to_home(robot)                                   # positions, zero overlay held
safe_switch(robot, "cartesian_impedance_controller")  # torque overlay
```

`scripts/kuka_crisp.py` also corrects two mismatches in crisp_py's stock `IiwaConfig`:
its `home_config` commands A4 to -135 deg against a +/-120 deg limit, and its
`base_frame` is `world`, which this description does not publish.
