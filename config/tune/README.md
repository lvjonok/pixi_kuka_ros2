# Cartesian impedance tuning (`scripts/track_tune.py`)

One recording, `~/data/track/rec1.jsonl` (26 Sep 2026, Haply teleop, 14.7 s; replayed to 11.4 s),
replayed under each gains file. Lag = the shift that best aligns arm with target.

| sweep | KUKA side (LBRServer joint impedance) | pick | pos lag / RMS | rot lag / RMS |
|---|---|---|---|---|
| 2 | stiffness 200, damping 0.7 (upstream default) | baseline 1300/84, 100/20 | 160 ms / 20.6 mm | 290 ms / 3.9 deg |
| 3 | same | k2500/d60 (k3000/d40 surged, then E-stop) | 90 ms / 13.5 mm | 200 ms / 3.5 deg |
| 5 | damping 0.3 (operator's recollection) | s4_r300_d10: 2500/60, 300/10, nullspace 30 | 65 ms / 12.2 mm | 75 ms / 1.4 deg |

Rotation above 100 needs `patches/crisp_controllers-rotational-stiffness-ceiling.patch`.
Sweep 5's baseline was 140 / 250 ms, so the KUKA damping explains part of the hidden lag; most of
the gain came from the stiffer rotation. Nullspace 10 instead of 30 did nothing or worse. In
rec1's last second the target asks A4 for ~80 % of its limit at any gains, so every run there
hits the 80 % abort; scores cover the first ~10.5 s.

**Adopted (26 Sep 2026):** `feel_k2500_d110_r250_d20_f09` -- 2500/110, 250/20, nullspace 30,
`filter.target_pose` 0.9 -- is now what `controllers_umi.yaml` launches. s4_r300_d10 replayed
faster but was erratic by hand and a rapid stroke tripped the driver's velocity guard. Use it as
`--restore` from now on; `baseline.yaml` is the pre-tuning launch.
Then damping raised for safety with force feedback on (d 110 -> 140, d_rot 20 -> 25):
`launch.yaml` mirrors what controllers_umi.yaml launches; use it as `--restore`.

## Small motions: sweeps 6-7 (26 Sep 2026, evening)

The first session with the real gripper found small hand movements dead. `track_tune.py fine`
scores what the lag average hides: the **hold** offset once the target has been still 1 s, and
the delay to half of each 0.5-10 mm target move. On the operator's own small moves (fine1) the
arm stuck 4-6 mm off a still target with no external force and stick-slipped. `steps1`
(`track_tune.py steps`: 1/3 mm and 0.5/2 deg steps about one pose) replayed:

| run | translation | rotation | hold p50 | step delay p50 / p90 |
|---|---|---|---|---|
| 6b launch | 2500/140 | 250/25 | 6.0 mm | 405 / 637 ms |
| 6b | 3500/170 | 350/30 | 4.8 mm | 140 / 440 ms |
| 6b | 5000/230 | 450/38 | 2.9 mm | 110 / 260 ms -- wrist oscillated on a 2 deg step, A3 81 %, aborted |
| 7 | **4000/200** | 250/25 | **3.5 mm** | **130 / 220 ms** |
| 7 | 5000/230 | 250/25 | 3.3 mm | 100 / 320 ms |

hold x k is ~15 N in every run: joint friction. At rest `/lbr/lbr_state` showed crisp's 1-2 Nm
per joint in the commanded torque and not in the measured one (external torque ~0), with the
commanded joint position equal to the measured -- the KUKA's joint spring holds nothing. A
stiffer spring breaks it sooner; nothing in crisp compensates it at standstill (`noise` is
declared but unimplemented, the friction model is velocity-driven with Franka values).

**Adopted:** 4000/200 translation, rotation unchanged -- `controllers_umi.yaml` and `launch.yaml`.
The next lever, if it is still sticky, is an integral term on the task error in crisp (clamped
to ~15 N, reset on re-engage).
