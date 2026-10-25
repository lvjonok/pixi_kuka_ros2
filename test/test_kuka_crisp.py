"""scripts/kuka_crisp.safe_switch's position_source refusal, against a fake robot: no ROS graph.

Only cartesian_impedance_controller tracks /lbr/target_pose, so no other overlay may run over a
passthrough that is not in `measured` (docs/position_source.md).
"""

from __future__ import annotations

from types import SimpleNamespace

import kuka_crisp as kc
import pytest

REST = {
    kc.TRAJECTORY: "active",
    kc.ZERO_EFFORT: "active",
    kc.PASSTHROUGH: "inactive",
    kc.CARTESIAN: "inactive",
    "joint_impedance_controller": "inactive",
}
ARMED = REST | {
    kc.TRAJECTORY: "inactive",
    kc.ZERO_EFFORT: "inactive",
    kc.PASSTHROUGH: "active",
    kc.CARTESIAN: "active",
}


class FakeSwitcher:
    def __init__(self, states):
        self.states = states
        self.best_effort: list[str] = []

    def get_controller_list(self):
        return [SimpleNamespace(name=n, state=s) for n, s in self.states.items()]

    def switch_controller(self, name, controllers_that_should_be_active):
        self.best_effort.append(name)
        return True


@pytest.fixture
def cell(monkeypatch):
    """A robot in ``states``, a passthrough at (parameter, running); records every switch."""
    strict: list[tuple[list[str], list[str]]] = []
    monkeypatch.setattr(
        kc,
        "strict_switch",
        lambda robot, activate, deactivate: strict.append((activate, deactivate)),
    )

    def make(states, parameter, running=None):
        monkeypatch.setattr(
            kc,
            "position_source_modes",
            lambda robot, armed: (parameter, running if armed else None),
        )
        robot = SimpleNamespace(controller_switcher_client=FakeSwitcher(states))
        return robot, strict

    return make


@pytest.mark.parametrize("mode", ["lagged", "ik_target"])
def test_rest_to_joint_impedance_is_refused_unless_measured(cell, mode):
    robot, strict = cell(REST, mode)
    with pytest.raises(RuntimeError, match=f"position_source is {mode}"):
        kc.safe_switch(robot, "joint_impedance_controller")
    assert strict == []


@pytest.mark.parametrize("mode", ["measured", "lagged", "ik_target"])
def test_rest_to_cartesian_arms_in_every_mode(cell, mode):
    robot, strict = cell(REST, mode)
    assert kc.safe_switch(robot, kc.CARTESIAN) is True
    assert strict == [([kc.PASSTHROUGH, kc.CARTESIAN], [kc.TRAJECTORY, kc.ZERO_EFFORT])]


def test_rest_to_joint_impedance_in_measured_or_on_an_old_build(cell):
    for parameter in ("measured", None):
        robot, strict = cell(REST, parameter)
        assert kc.safe_switch(robot, "joint_impedance_controller") is True
    assert len(strict) == 2


@pytest.mark.parametrize(
    "parameter, running, needle",
    [
        ("ik_target", "ik_target", "position_source is ik_target"),
        # Set back with a plain `ros2 param set` while armed: the active mode is what counts.
        ("measured", "ik_target", "running ik_target"),
        ("measured", None, "unknown mode"),
    ],
)
def test_armed_to_joint_impedance_is_refused_unless_running_measured(
    cell, parameter, running, needle
):
    robot, _ = cell(ARMED, parameter, running)
    with pytest.raises(RuntimeError, match=needle):
        kc.safe_switch(robot, "joint_impedance_controller")
    assert robot.controller_switcher_client.best_effort == []


def test_armed_switches_that_stay_allowed(cell):
    robot, strict = cell(ARMED, "measured", "measured")
    assert kc.safe_switch(robot, "joint_impedance_controller") is True
    robot, strict = cell(ARMED, "ik_target", "ik_target")
    assert kc.safe_switch(robot, kc.CARTESIAN) is True
    # ARMED -> REST always works.
    assert kc.safe_switch(robot, kc.ZERO_EFFORT) is True
    assert strict[-1][0] == [kc.TRAJECTORY, kc.ZERO_EFFORT]
