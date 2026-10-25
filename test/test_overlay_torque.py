"""scripts/overlay_torque.py's arithmetic on synthetic LBRState arrays (no bag, no ROS)."""

import numpy as np
from overlay_torque import summarize


def test_overlay_and_spring_per_joint():
    n = 200
    tau = np.tile([2.0, -2.0, 0.0, 1.0, 0.0, 0.0, 0.5], (n, 1))
    q_meas = np.zeros((n, 7))
    q_cmd = q_meas.copy()
    q_cmd[:, 0] = 0.01  # A1 commanded 10 mrad ahead: 2 Nm from a 200 Nm/rad spring
    s = summarize(tau, q_cmd, q_meas, np.full(7, 200.0))
    assert np.allclose(s["overlay_rms"], np.abs(tau[0]))
    assert np.allclose(s["spring_rms"], [2.0, 0, 0, 0, 0, 0, 0])
    assert "modes" not in s  # no ~/state in the bag: no clamp fractions


def test_clamp_fractions_from_the_passthrough_state():
    rows = np.zeros((4, 37))
    rows[:, 0] = 2
    rows[:2, 23] = 1  # A1 gap-clamped on half the updates
    rows[3, 36] = 1  # A7 step-clamped once
    rows[0, 1] = 1
    z = np.zeros((4, 7))
    s = summarize(z, z, z, np.full(7, 200.0), rows)
    assert s["modes"].tolist() == [2.0]
    assert s["gap_clamped_frac"][0] == 0.5 and s["step_clamped_frac"][6] == 0.25
    assert s["stale_frac"] == 0.25
