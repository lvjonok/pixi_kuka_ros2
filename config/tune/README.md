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
