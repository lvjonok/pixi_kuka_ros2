# Running an autonomous excitation session

Recording free-space data for the NEXT torque model by driving the arm through a planned
trajectory instead of teleoperating it.

**This is a different risk class from everything else in this repo.** Every other motion here
happens with a hand on the clutch. This one does not. Read [Before you start](#before-you-start)
and do not skip the dry run.

## Why

Teleoperation is a poor excitation source, measured on `shakedown2` (5.4 min):

| | teleop | what the plan achieves |
|---|---|---|
| least-moved joint travel | **5.7 %** of its range | 84 % |
| participation ratio of visited configurations | **3.49** of 7 | 5.71 |
| velocity sign reversals, worst joint | 15 / min | 24 / min |
| `tau_m` range on A5 / A6 / A7 | 1.5 / 1.5 / 0.8 Nm | — |
| tip bounding box | 8.1 L of a 157.5 L box | — |

The participation ratio has a single cause: `q_ref`, the controller's nullspace target, had a
range of **exactly 0.0000 rad on all seven joints** for the whole session. A Cartesian controller
commands six degrees of freedom and pins the seventh, so the elbow never moved. Teleoperating
harder does not fix that — it is a property of the control law.

## The shape of it

```
  laptop                                    bench
  ------                                    -----
  iiwa-next excite  --->  plan.npz  --->  excite_player.py
    plans in joint space                    |  UDP, same packet the Haply sends
    gates: 6 safety + 4 coverage            v
    replays in viser                      haply_teleop_bridge.py
                                            |  watchdog, slew clamps, workspace box,
                                            |  error clamp (50 mm x 500 N/m = 25 N)
                                            v
                                          /lbr/target_pose   (task: 6 DOF)
                                          /lbr/target_joint  (nullspace: the 7th)
```

The player is a **drop-in for the Haply**. It speaks the bridge's packet format and nothing else,
so the safety envelope is the one already proven on this cell with a person holding the clutch.
No part of it is reimplemented for autonomous use — a second copy of a safety envelope is a
second thing to get wrong, and the copy that only runs unattended is the one nobody exercises.

The packet gained one optional field, `joints`. When present the bridge republishes it on
`/lbr/target_joint`, slew-clamped at `--max-joint-deg-s` (45 °/s) and held inside the joint
limits by `--joint-margin-rad` (0.15 rad). Teleop never sends it, so nothing about teleop changes.

## Before you start

- [ ] **The cell is clear.** The arm sweeps most of the workspace box. Nobody reaches in.
- [ ] **You are at the e-stop** for the whole session, not in the next room.
- [ ] **Payload is what the plan assumed** — bare flange, `payload_kg: 0.0`.
- [ ] **You have watched this exact plan in viser**, at wall-clock speed. A plan that has not
      been watched has not been reviewed: every gate check is a number, and no number catches an
      arm sweeping through where a person stands.
- [ ] **Dry run first.** `--dry-run` does everything except send.

## 1. Bring the arm to a start pose, supervised

The plan begins where the arm already is, so this step is *not* the plan's job — it is the one
motion a person drives.

```bash
cd ~/github.com/lvjonok/hardware/pixi_kuka_ros2
pixi run -e jazzy python scripts/crisp_session.py --home
```

`HOME_DEGREES` is `[0, 30, 0, -75, 0, 75, 0]`, which puts the tip at **(0.596, 0.000, 0.494)** —
inside the workspace box, manipulability 0.126, nullspace not stalled. Any pose inside the box
works; home is simply known-good.

Read the joints back. **Do not assume them** — read them, every session:

```bash
pixi run -e jazzy ros2 topic echo --once /lbr/joint_states
```

## 2. Plan and gate it (laptop)

```bash
cd ~/github.com/lvjonok/iiwa_next
pixi run -e dev iiwa-next excite \
    --urdf   <iiwa14.urdf> \
    --mesh-dir <.../lbr_iiwa14_r820_description/share> \
    --srdf   <.../iiwa14_moveit_config/config/iiwa14.srdf> \
    --start-deg 0 30 0 -75 0 75 0 \
    --scaffolds 160 --seed 1 \
    --out ~/data/excite_01.npz --replay
```

`--srdf` is not optional in practice: without it every adjacent link pair reads as a
self-collision and the sampler rejects roughly 98 % of candidates. Without `--mesh-dir`,
self-collision **cannot be checked at all**, and the gate treats that as fatal rather than as a
pass — a plan running unattended must not treat an unanswered safety question as an answered one.

The gate prints every check with its measurement and ends in `PLAN ACCEPTED` or `PLAN REJECTED`.
A rejected plan is not written and not replayed.

It also prints an `excitation` block. Paste it into the recording's session JSON — it carries the
plan's content hash, which is what later makes "did the elbow actually follow?" answerable from
the data rather than from memory.

## 3. Start the stack and the bridge

Three panes, in order:

```bash
# 1. hardware
pixi run hardware

# 2. switch to the Cartesian controller (crisp_session, then let it exit)
pixi run -e jazzy python scripts/crisp_session.py --home

# 3. the bridge
pixi run -e jazzy python scripts/haply_teleop_bridge.py
```

Watch pane 1 as well as pane 3. `CartesianController` refuses **every** command on a topic
carrying more than one publisher and says so only in the controller_manager's log — from the
bridge's side that looks like a healthy bridge publishing into an arm that ignores it. The bridge
checks both command topics at startup and shouts if it finds another publisher; the usual cause
is an orphaned `crisp_py` process.

## 4. Dry run, then run

```bash
cd ~/github.com/lvjonok/hardware/pixi_kuka_ros2

# refuses unless the arm is already at the plan's first configuration
pixi run -e jazzy python scripts/excite_player.py ~/data/excite_01.npz \
    --urdf <iiwa14.urdf> --dry-run

# then, for real, slowly the first time
pixi run -e jazzy python scripts/excite_player.py ~/data/excite_01.npz \
    --urdf <iiwa14.urdf> --speed 0.25
```

The player ramps in over the first 3 s regardless of what the plan says, and asks for a typed
confirmation before it sends anything.

**To stop: Ctrl-C.** The player stops publishing and explicitly releases; the bridge's 100 ms
watchdog then holds. Under impedance "stop commanding" is a hold, not a release — the arm keeps
its last commanded pose rather than going limp.

## 5. Record it

Start the recorder before the player, in its own pane:

```bash
pixi run -e jazzy iiwa-next record --out ~/data/excite_01.parquet --session session.json
```

with the `excitation` block from step 2 in `session.json`. Then gate the recording:

```bash
pixi run -e jazzy iiwa-next validate ~/data/excite_01.parquet
```

## Afterwards: did the elbow actually follow?

This is the open question the plan cannot answer by itself. In **58 % of sampled configurations**
gravity projected on the elbow direction exceeds the 3 Nm the Cartesian controller's nullspace
term can push with (`nullspace.stiffness: 5.0`, `max_tau: 3.0` in `config/controllers.yaml`). If
the elbow does not follow `target_joint`, the plan promises a participation ratio of 5.71 and the
recording delivers something closer to teleop's 3.49 — and it will look perfectly healthy either
way.

The recording answers it. Compare the recorded `q_ref` against the plan:

- `q_ref` moving with a range well above zero means the nullspace is being commanded at all
  (it was flat 0.0000 for the whole teleop session);
- `q_ref` tracking the plan means the authority is sufficient;
- `q_ref` moving but the *measured* `q` not following it means 3 Nm is not enough, and the gain
  is the thing to change — measured, with the number beside it.

## Known gaps

- **Sign reversals sit at 24/min against the 30/min the gate asks for.** Coulomb friction is
  identified by direction changes at matched speed and by nothing else. Shorter dither bursts or
  more harmonics would raise it; not yet tuned.
- **`q_ref` moving inside the controller has not been verified end to end.** `/lbr/target_joint`
  was confirmed published with correct joint names against the mock, and the controller's
  subscription to it is in `cartesian_controller.cpp`, but the introspection read-back of `q_ref`
  on the mock could not be made to deliver. First real session settles it.
- **FRI reports no joint temperature.** Friction changes 17–54 % over a warm-up with the sign of
  the drift depending on speed, so the plan interleaves conditions rather than sweeping them and
  re-runs a fixed probe burst every 15 minutes. That probe's drift across the session is the only
  thermal measurement available; it is in the recording, labelled `probe`.
