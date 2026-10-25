"""scripts/position_source.py's decisions, against a fake cell: no ROS graph, nothing moves.

.pixi/envs/jazzy/bin/python -m pytest test      # or: pixi run -e jazzy pytest test
"""

from __future__ import annotations

import math

import position_source as ps
import pytest
from position_source import Param

REST = {
    "joint_state_broadcaster": "active",
    "joint_trajectory_controller": "active",
    "zero_effort_controller": "active",
    "fri_position_passthrough_controller": "inactive",
    "cartesian_impedance_controller": "inactive",
    "joint_impedance_controller": "inactive",
}
ARMED = REST | {
    "joint_trajectory_controller": "inactive",
    "zero_effort_controller": "inactive",
    "fri_position_passthrough_controller": "active",
    "cartesian_impedance_controller": "active",
}


def defaults() -> dict[str, Param]:
    """The passthrough's parameters as docs/position_source.md declares them."""
    return {
        "position_source": Param("string", "measured"),
        "lag_time_constant_s": Param("double", 0.008),
        "target_topic": Param("string", "target_pose"),
        "end_effector_frame": Param("string", "lbr_link_ee"),
        "base_frame": Param("string", "lbr_link_0"),
        "target_filter": Param("double", 0.9),
        "target_timeout_s": Param("double", 0.1),
        "ik_damping": Param("double", 0.01),
        "nullspace_gain": Param("double", 0.05),
        "joint_limit_margin_rad": Param("double", 0.05),
        "max_joint_step_rad": Param("double_array", [0.004] * 7),
        "sunrise_joint_stiffness": Param("double_array", [200.0] * 7),
        "max_spring_torque_nm": Param("double_array", [10.0, 10.0, 10.0, 8.0, 5.0, 5.0, 3.0]),
    }


def crisp(frame: str = "lbr_link_ee") -> dict[str, Param]:
    return {
        "end_effector_frame": Param("string", frame),
        "base_frame": Param("string", "lbr_link_0"),
        "topics.target_pose": Param("string", "target_pose"),
        "filter.target_pose": Param("double", 0.9),
    }


class FakeCell:
    """The four RosCell methods over dictionaries; records every set."""

    def __init__(self, states=REST, params=None, cartesian=None, state_msg=None):
        self.states = [dict(states)]  # one per controller_states() call; the last repeats
        self.nodes = {ps.PASSTHROUGH: defaults() if params is None else params}
        if cartesian is not None:
            self.nodes[ps.CARTESIAN] = cartesian
        self.state_msg = state_msg
        self.sets: list[dict[str, Param]] = []
        self.refuse: str | None = None  # the controller's reason, if it refuses
        self.ignore_sets = False  # accepts but keeps the old values (a broken readback)

    def controller_states(self):
        return self.states.pop(0) if len(self.states) > 1 else self.states[0]

    def get_parameters(self, node, names):
        if node not in self.nodes:
            return None
        return {n: self.nodes[node][n] for n in names if n in self.nodes[node]}

    def set_parameters(self, node, params):
        self.sets.append(dict(params))
        if self.refuse is not None:
            return self.refuse
        if not self.ignore_sets:
            self.nodes[node] |= params
        return None

    def read_state(self, timeout_s):
        return self.state_msg


def state_msg(mode: int, stale: int = 0, gap=(0,) * 7, step=(0,) * 7) -> list[float]:
    q = [0.1 * j for j in range(7)]
    return [float(mode), float(stale), *q, *q, *q, *map(float, gap), *map(float, step)]


# -- REST ------------------------------------------------------------------------------------


def test_cell_state_names_rest_and_armed():
    assert ps.cell_state(REST) == "REST"
    assert ps.cell_state(ARMED) == "ARMED (cartesian_impedance_controller)"
    # passthrough + zero effort, the old resting pair that holds nothing (kuka_crisp.safe_switch)
    old = REST | {
        "joint_trajectory_controller": "inactive",
        "fri_position_passthrough_controller": "active",
    }
    assert ps.cell_state(old).startswith("neither REST nor ARMED")


@pytest.mark.parametrize(
    "states, needle",
    [
        (ARMED, "is active"),
        (REST | {"zero_effort_controller": "inactive"}, "zero_effort_controller not active"),
        (REST | {"joint_trajectory_controller": "inactive"}, "joint_trajectory_controller"),
        ({k: v for k, v in REST.items() if k != ps.PASSTHROUGH}, "not loaded"),
    ],
)
def test_not_rest_is_refused_and_nothing_is_set(states, needle, capsys):
    cell = FakeCell(states=states)
    assert ps.rest_problem(states) is not None and needle in ps.rest_problem(states)
    assert ps.cmd_set(cell, "lagged", []) == 1
    assert cell.sets == []
    assert "REFUSED" in capsys.readouterr().err


def test_set_at_rest_sets_reads_back_and_says_next_arm(capsys):
    cell = FakeCell()
    assert ps.cmd_set(cell, "lagged", ["lag_time_constant_s=0.01"]) == 0
    assert cell.sets == [
        {"position_source": Param("string", "lagged"), "lag_time_constant_s": Param("double", 0.01)}
    ]
    out = capsys.readouterr().out
    assert "read back: identical" in out
    assert "next arm" in out


def test_leaving_rest_during_the_set_is_reported(capsys):
    cell = FakeCell()
    cell.states = [dict(REST), dict(ARMED)]
    assert ps.cmd_set(cell, "lagged", []) == 1
    assert "left REST" in capsys.readouterr().err


# -- what set accepts ------------------------------------------------------------------------


@pytest.mark.parametrize(
    "item, needle",
    [
        ("lag_time_constant_s", "expected name=value"),
        ("lag_time_constnat_s=0.01", "not a position_source parameter"),
        ("position_source=ik_target", "positional argument"),
    ],
)
def test_bad_assignments_are_refused_before_touching_the_cell(item, needle, capsys):
    cell = FakeCell()
    assert ps.cmd_set(cell, "lagged", [item]) == 1
    assert cell.sets == []
    assert needle in capsys.readouterr().err


def test_a_parameter_given_twice_is_refused():
    with pytest.raises(ValueError, match="twice"):
        ps.parse_assignments(["ik_damping=0.1", "ik_damping=0.2"])


def test_unknown_mode_is_refused():
    assert ps.cmd_set(FakeCell(), "drake", []) == 1


def test_array_parameters_take_one_value_or_seven():
    live = Param("double_array", [0.004] * 7)
    assert ps.coerce("max_joint_step_rad", "0.002", live).value == [0.002] * 7
    seven = ps.coerce("max_spring_torque_nm", "[5, 5, 5, 4, 3, 3, 2]", live)
    assert seven == Param("double_array", [5.0, 5.0, 5.0, 4.0, 3.0, 3.0, 2.0])
    with pytest.raises(ValueError, match="3 values"):
        ps.coerce("max_spring_torque_nm", "1,2,3", live)


def test_a_per_joint_step_limit_is_accepted_comma_separated(capsys):
    cell = FakeCell()
    steps = "0.004,0.004,0.004,0.004,0.003,0.003,0.002"
    assert ps.cmd_set(cell, "lagged", [f"max_joint_step_rad={steps}"]) == 0
    assert cell.sets[0]["max_joint_step_rad"] == Param(
        "double_array", [0.004] * 4 + [0.003] * 2 + [0.002]
    )
    assert "read back: identical" in capsys.readouterr().out


def test_the_live_type_decides():
    # A scalar YAML override declares a double (the type the file gave it): set as a double.
    assert ps.coerce("max_joint_step_rad", "0.003", Param("double", 0.004)) == Param(
        "double", 0.003
    )
    with pytest.raises(ValueError, match="max_joint_step_rad"):
        ps.coerce("max_joint_step_rad", "[0.004,0.003]", Param("double", 0.004))
    with pytest.raises(ValueError, match="not finite"):
        ps.coerce("ik_damping", "nan", Param("double", 0.01))
    with pytest.raises(ValueError):
        ps.coerce("ik_damping", "lots", Param("double", 0.01))


def test_an_old_controller_without_position_source_is_refused(capsys):
    cell = FakeCell(params={})
    assert ps.cmd_set(cell, "lagged", []) == 1
    assert cell.sets == []
    assert "not declared" in capsys.readouterr().err


@pytest.mark.parametrize(
    "item",
    [
        "target_filter=1.0",
        "sunrise_joint_stiffness=0",
        "target_timeout_s=0",
        "nullspace_gain=2",
        "lag_time_constant_s=-1",
        "end_effector_frame=",
    ],
)
def test_values_the_controller_would_fail_activation_on_are_refused(item, capsys):
    cell = FakeCell()
    assert ps.cmd_set(cell, "lagged", [item]) == 1
    assert cell.sets == []


def test_a_refusal_by_the_controller_fails(capsys):
    cell = FakeCell()
    cell.refuse = "parameter is read-only"
    assert ps.cmd_set(cell, "lagged", []) == 1
    assert "read-only" in capsys.readouterr().err


def test_a_readback_that_differs_fails(capsys):
    cell = FakeCell()
    cell.ignore_sets = True
    assert ps.cmd_set(cell, "lagged", []) == 1
    assert "read back differs for position_source (measured)" in capsys.readouterr().err


# -- ik_target -------------------------------------------------------------------------------


def test_ik_target_needs_crisp_frames(capsys):
    cell = FakeCell(cartesian=None)
    assert ps.cmd_set(cell, "ik_target", []) == 1
    assert cell.sets == []
    assert "cannot read cartesian_impedance_controller" in capsys.readouterr().err


def test_ik_target_refuses_a_frame_crisp_is_not_driving(capsys):
    # crisp drives lbr_umi_camera (tool:=umi) while the passthrough was left on lbr_link_ee.
    cell = FakeCell(cartesian=crisp("lbr_umi_camera"))
    assert ps.cmd_set(cell, "ik_target", []) == 1
    assert cell.sets == []
    assert "--param end_effector_frame=lbr_umi_camera" in capsys.readouterr().err
    assert ps.cmd_set(cell, "ik_target", ["end_effector_frame=lbr_umi_camera"]) == 0


def test_ik_target_refuses_another_target_topic():
    cell = FakeCell(cartesian=crisp())
    assert ps.cmd_set(cell, "ik_target", ["target_topic=other_pose"]) == 1
    assert cell.sets == []


def test_ik_target_warns_on_a_different_filter(capsys):
    cell = FakeCell(cartesian=crisp())
    assert ps.cmd_set(cell, "ik_target", ["target_filter=0.5"]) == 0
    assert "filter the same target differently" in capsys.readouterr().out


def test_frames_are_not_checked_outside_ik_target():
    assert ps.cmd_set(FakeCell(cartesian=crisp("lbr_umi_camera")), "lagged", []) == 0


def test_stiffer_springs_are_only_warned_about_under_measured(capsys):
    # measured uses no limit, so `set measured` always works; the warning names what is loose.
    cell = FakeCell()
    assert (
        ps.cmd_set(cell, "measured", ["sunrise_joint_stiffness=500", "max_spring_torque_nm=20"])
        == 0
    )
    out = capsys.readouterr().out
    assert "commissioning_protocol.md" in out
    assert "the gap clamp is the only guard" in out
    assert "refused for lagged and ik_target" in out
    # ...and the next other mode is refused on the values now live.
    assert ps.cmd_set(cell, "lagged", []) == 1
    assert len(cell.sets) == 1


@pytest.mark.parametrize("mode", ["lagged", "ik_target"])
@pytest.mark.parametrize(
    "item, needle",
    [
        # A typo for 200: a 0.5 rad gap, 100 Nm from the stock app's spring.
        ("sunrise_joint_stiffness=20", "a value below the app's widens the gap"),
        ("max_spring_torque_nm=200", "exceeds the spec's"),
        ("max_joint_step_rad=1.0", "max_joint_step_rad = [1, 1, 1, 1, 1, 1, 1] exceeds"),
        ("'max_spring_torque_nm=[10,10,10,8,5,5,4]'", "on A7"),
    ],
)
def test_a_looser_limit_is_refused_without_review(mode, item, needle, capsys):
    cell = FakeCell(cartesian=crisp())
    assert ps.cmd_set(cell, mode, [item.strip("'")]) == 1
    assert cell.sets == []
    err = capsys.readouterr().err
    assert needle in err and "--limit-change-reviewed" in err


def test_a_reviewed_limit_change_prints_the_bound_before_setting(capsys):
    cell = FakeCell(cartesian=crisp())
    params = ["sunrise_joint_stiffness=500", "max_spring_torque_nm=[20,20,20,16,10,10,6]"]
    assert ps.cmd_set(cell, "ik_target", params, limit_change_reviewed=True) == 0
    out = capsys.readouterr().out
    assert out.index("LIMIT CHANGE") < out.index("set on /lbr/")
    assert "|q_out - q_meas| <= [0.04, 0.04, 0.04, 0.032, 0.02, 0.02, 0.012] rad" in out
    assert "may add up to [20, 20, 20, 16, 10, 10, 6] Nm" in out


def test_tighter_limits_need_no_review():
    params = ["max_spring_torque_nm=[5,5,5,4,3,3,2]", "max_joint_step_rad=0.002"]
    assert ps.cmd_set(FakeCell(), "lagged", params) == 0


def test_the_spring_gap_is_torque_over_stiffness():
    values = {n: p.value for n, p in defaults().items()}
    gap = ps.spring_gap(values)
    assert gap[:3] == [0.05] * 3  # the spec's 0.05 rad on A1-A3 at 200 Nm/rad
    assert math.isclose(gap[6], 3.0 / 200.0)


# -- status and ~/state ----------------------------------------------------------------------


def test_parse_state_reads_the_listed_layout():
    s = ps.parse_state(state_msg(2, stale=1, gap=(1, 0, 0, 0, 0, 0, 1), step=(0, 1, 0, 0, 0, 0, 0)))
    assert s["mode"] == "ik_target" and s["stale"]
    assert s["gap_clamped"] == ["A1", "A7"] and s["step_clamped"] == ["A2"]
    assert s["q_out"] == [0.1 * j for j in range(7)]
    assert len(state_msg(0)) == 37  # 2 + 5 x 7
    assert ps.parse_state(state_msg(7))["mode"].startswith("unknown")
    assert ps.parse_state([]) is None


def test_another_layout_still_gives_the_mode_but_no_clamps(capsys):
    other = [1.0, 0.0] + [0.0] * 42
    assert ps.parse_state(other) == {"mode": "lagged", "stale": False, "size": 44}
    params = defaults() | {"position_source": Param("string", "lagged")}
    assert ps.cmd_status(FakeCell(states=ARMED, params=params, state_msg=other)) == 0
    out = capsys.readouterr().out
    assert "running (~/state): lagged" in out and "clamps not shown: 44 values" in out
    assert "differs" not in out


def test_status_at_rest_lists_the_parameters(capsys):
    assert ps.cmd_status(FakeCell()) == 0
    out = capsys.readouterr().out
    assert "cell: REST" in out
    assert "position_source" in out and "measured" in out
    assert "next arm" in out


def test_status_armed_reports_the_running_mode_and_a_pending_change(capsys):
    params = defaults() | {"position_source": Param("string", "ik_target")}
    cell = FakeCell(states=ARMED, params=params, state_msg=state_msg(0))
    assert ps.cmd_status(cell) == 0
    out = capsys.readouterr().out
    assert "running (~/state): measured" in out
    assert "differs from the running mode" in out


def test_status_of_an_old_controller(capsys):
    assert ps.cmd_status(FakeCell(params={})) == 0
    assert "predates" in capsys.readouterr().out


def test_the_parser_takes_only_the_three_modes():
    parser = ps.build_parser()
    args = parser.parse_args(["set", "ik_target", "--param", "a=1", "--param", "b=2"])
    assert (args.mode, args.param) == ("ik_target", ["a=1", "b=2"])
    with pytest.raises(SystemExit):
        parser.parse_args(["set", "drake"])
