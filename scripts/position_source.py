"""Read, and at REST set, what fri_position_passthrough_controller writes into FRI's position field.

docs/position_source.md is the specification. In TORQUE client command mode FRI carries a joint
position and a torque every cycle, and Sunrise's joint impedance spring is centred on that
position. `position_source` chooses it:

* measured  -- the measured position, as the controller always did: Sunrise's spring holds
  nothing and crisp's overlay holds the arm. The default.
* lagged    -- a first-order low-pass of the measured position (lag_time_constant_s), as Drake's
  torque-only mode: a small gap that trails any motion, which is what KUKA's friction
  compensation reportedly needs to engage.
* ik_target -- the differential IK of crisp's own /lbr/target_pose, clamped per joint to within
  max_spring_torque_nm / sunrise_joint_stiffness of the measured position: Sunrise's spring then
  pulls toward the target as well.

The controller reads these parameters at activation, so a change takes effect at the next
REST -> ARMED. `set` therefore refuses unless the cell is at REST as scripts/kuka_crisp.py
defines it -- joint_trajectory_controller + zero_effort_controller active, the passthrough
inactive -- so a change can never land on the controller that is writing the position field.

    pixi run -e jazzy position-source status
    pixi run -e jazzy position-source set lagged
    pixi run -e jazzy position-source set ik_target --param target_timeout_s=0.2
    pixi run -e jazzy position-source set measured   # the old behaviour, before any yubi session

A value that loosens the guard on Sunrise's spring torque beyond the spec's defaults (a wider gap,
a stiffness other than the stock 200, a larger max_spring_torque_nm or max_joint_step_rad) is
refused for lagged and ik_target unless --limit-change-reviewed is given: that is a limit change
under docs/commissioning_protocol.md, not a tuning step.

Moves nothing and switches no controller. The parameters go through the controller's
set_parameters_atomically service (all or nothing), are read back, and the next arm
(`session --switch cartesian_impedance_controller`, or yubi's `make -C deploy cell-arm`) logs the
mode it activated with. A relaunch of the stack returns to config/controllers.yaml.

The decision logic takes any object with the four methods of RosCell, so tests/ drives it with
fakes and no ROS graph.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Any, Protocol

NS = "/lbr"
PASSTHROUGH = "fri_position_passthrough_controller"
TRAJECTORY = "joint_trajectory_controller"
ZERO_EFFORT = "zero_effort_controller"
CARTESIAN = "cartesian_impedance_controller"
TORQUE_CONTROLLERS = ("cartesian_impedance_controller", "joint_impedance_controller")
JOINTS = 7

# The ~/state layout's first value (docs/position_source.md, "Diagnostics").
MODES = ("measured", "lagged", "ik_target")
# [mode, stale, q_meas[7], q_ik[7], q_out[7], gap_clamped[7], step_clamped[7]]: 37 values; mode
# and stale are read from any length.
STATE_SIZE = 2 + 5 * JOINTS

# Every parameter docs/position_source.md adds, in its table's order; `set --param` takes only
# these, so a typo is refused here instead of being a silently ignored undeclared name.
PARAMETERS = (
    "position_source",
    "lag_time_constant_s",
    "target_topic",
    "end_effector_frame",
    "base_frame",
    "target_filter",
    "target_timeout_s",
    "ik_damping",
    "nullspace_gain",
    "joint_limit_margin_rad",
    "max_joint_step_rad",
    "sunrise_joint_stiffness",
    "max_spring_torque_nm",
)

# What ik_target must agree with crisp on: the frame it solves for and the target it reads.
CRISP_PARAMETERS = ("end_effector_frame", "base_frame", "topics.target_pose", "filter.target_pose")

# The spec's defaults for the numbers that bound Sunrise's spring torque: the gap clamp's two
# and the output's step limit. Anything looser is a limit change (docs/commissioning_protocol.md):
# `set` refuses it for the modes that use them unless --limit-change-reviewed is given.
DEFAULT_STIFFNESS = (200.0,) * JOINTS
DEFAULT_SPRING_TORQUE = (10.0, 10.0, 10.0, 8.0, 5.0, 5.0, 3.0)
DEFAULT_GAP = tuple(t / k for t, k in zip(DEFAULT_SPRING_TORQUE, DEFAULT_STIFFNESS))
DEFAULT_MAX_STEP = 0.004


@dataclass(frozen=True)
class Param:
    """A parameter value as the ROS parameter services carry it: its type name and the value."""

    type: str  # "bool", "integer", "double", "string", "double_array", ..., "not_set"
    value: Any


class Cell(Protocol):
    def controller_states(self) -> dict[str, str]: ...

    def get_parameters(self, node: str, names: tuple[str, ...]) -> dict[str, Param] | None:
        """The declared ones among ``names``; None if the node's services are not up."""

    def set_parameters(self, node: str, params: dict[str, Param]) -> str | None:
        """Set all of them or none; the refusal's reason, or None."""

    def read_state(self, timeout_s: float) -> list[float] | None: ...


# -- decisions (no ROS) ---------------------------------------------------------------------


def cell_state(states: dict[str, str]) -> str:
    """REST, ARMED (<overlay>), or a description of whatever else the cell is in."""
    active = {name for name, state in states.items() if state == "active"}
    if {TRAJECTORY, ZERO_EFFORT} <= active and PASSTHROUGH not in active:
        return "REST"
    overlays = sorted(set(TORQUE_CONTROLLERS) & active)
    if PASSTHROUGH in active and overlays:
        return f"ARMED ({', '.join(overlays)})"
    owners = sorted({PASSTHROUGH, TRAJECTORY, ZERO_EFFORT, *TORQUE_CONTROLLERS} & active)
    return f"neither REST nor ARMED (active: {', '.join(owners) or 'none of the command owners'})"


def rest_problem(states: dict[str, str]) -> str | None:
    """Why the cell is not at REST, or None.

    The passthrough must be loaded but not active: its parameters are only read at activation,
    and only a REST cell is guaranteed to activate it next through one STRICT switch.
    """
    if PASSTHROUGH not in states:
        return f"{PASSTHROUGH} is not loaded: is this branch's stack running?"
    if states[PASSTHROUGH] == "active":
        return f"{PASSTHROUGH} is active (the cell is {cell_state(states)}): REST it first"
    missing = [c for c in (TRAJECTORY, ZERO_EFFORT) if states.get(c) != "active"]
    if missing:
        return f"not at REST: {', '.join(missing)} not active (the cell is {cell_state(states)})"
    return None


def parse_assignments(items: list[str]) -> dict[str, str]:
    """``["name=value", ...]`` -> ``{name: value}``; refuses what `set` must not take."""
    out: dict[str, str] = {}
    for item in items:
        name, sep, text = item.partition("=")
        name = name.strip()
        if not sep or not name:
            raise ValueError(f"--param {item!r}: expected name=value")
        if name == "position_source":
            raise ValueError("position_source is the positional argument of `set`, not a --param")
        if name not in PARAMETERS:
            raise ValueError(
                f"--param {name}: not a position_source parameter ({', '.join(PARAMETERS[1:])})"
            )
        if name in out:
            raise ValueError(f"--param {name} given twice")
        out[name] = text.strip()
    return out


def _finite(text: str) -> float:
    value = float(text)
    if not math.isfinite(value):
        raise ValueError(f"{text!r} is not finite")
    return value


def coerce(name: str, text: str, live: Param | None) -> Param:
    """``text`` as the type the running controller declared ``name`` with.

    The live type decides, not this file: a YAML override fixes it before the controller declares
    the parameter. A single number for an array parameter is given to every joint.
    """
    if live is None or live.type == "not_set":
        raise ValueError(f"{name} is not declared by the running {PASSTHROUGH}")
    try:
        return _coerce(name, text, live)
    except ValueError as why:
        raise ValueError(f"{name} ({live.type}): {why}") from None


def _coerce(name: str, text: str, live: Param) -> Param:
    if live.type == "string":
        return Param("string", text)
    if live.type == "double":
        return Param("double", _finite(text))
    if live.type == "integer":
        return Param("integer", int(text))
    if live.type == "bool":
        if text.lower() not in ("true", "false"):
            raise ValueError(f"{name}: {text!r} is not true/false")
        return Param("bool", text.lower() == "true")
    if live.type == "double_array":
        parts = [p for p in text.strip("[]").replace(",", " ").split() if p]
        values = [_finite(p) for p in parts]
        n = len(live.value) or JOINTS
        if len(values) == 1:
            values = values * n
        if len(values) != n:
            raise ValueError(f"{name}: {len(values)} values, the controller holds {n}")
        return Param("double_array", values)
    raise ValueError(f"{name}: cannot set a {live.type} parameter from the command line")


def _per_joint(value: Any) -> list[float]:
    return (
        [float(v) for v in value] if isinstance(value, (list, tuple)) else [float(value)] * JOINTS
    )


def validate(values: dict[str, Any]) -> list[str]:
    """What is wrong with these (final) values; the controller would fail activation on most."""
    problems = []

    def check(name: str, ok: Any, why: str) -> None:
        if name in values:
            try:
                good = ok(values[name])
            except (TypeError, ValueError):
                good = False
            if not good:
                problems.append(f"{name} = {values[name]!r}: {why}")

    check("position_source", lambda v: v in MODES, f"must be one of {', '.join(MODES)}")
    check("lag_time_constant_s", lambda v: v >= 0, "must be >= 0")
    check("target_filter", lambda v: 0 <= v < 1, "must be in [0, 1); 1 freezes the target")
    check("target_timeout_s", lambda v: v > 0, "must be > 0")
    check("ik_damping", lambda v: v >= 0, "must be >= 0")
    check("nullspace_gain", lambda v: 0 <= v <= 1, "must be in [0, 1] (a per-update fraction)")
    check("joint_limit_margin_rad", lambda v: v >= 0, "must be >= 0")
    check("max_joint_step_rad", lambda v: all(x > 0 for x in _per_joint(v)), "must be > 0")
    check("sunrise_joint_stiffness", lambda v: all(x > 0 for x in _per_joint(v)), "must be > 0")
    check("max_spring_torque_nm", lambda v: all(x >= 0 for x in _per_joint(v)), "must be >= 0")
    for name in ("target_topic", "end_effector_frame", "base_frame"):
        check(name, lambda v: isinstance(v, str) and v.strip() != "", "must not be empty")
    return problems


def spring_gap(values: dict[str, Any]) -> list[float] | None:
    """Per joint, the largest |q_out - q_meas| the clamp allows, rad."""
    if "sunrise_joint_stiffness" not in values or "max_spring_torque_nm" not in values:
        return None
    k = _per_joint(values["sunrise_joint_stiffness"])
    tau = _per_joint(values["max_spring_torque_nm"])
    # A non-positive stiffness is refused by validate(); unbounded here.
    return [t / s if s > 0 else math.inf for t, s in zip(tau, k)]


def ik_target_checks(
    values: dict[str, Any], crisp: dict[str, Param] | None
) -> tuple[list[str], list[str]]:
    """(problems, warnings) for ik_target: it must solve for the frame crisp is driving."""
    problems: list[str] = []
    warnings: list[str] = []
    if crisp is None:
        return [f"cannot read {CARTESIAN}'s parameters: ik_target must use its frames"], warnings
    for name in ("end_effector_frame", "base_frame"):
        theirs = crisp.get(name)
        if theirs is None:
            problems.append(f"{CARTESIAN} has no {name} parameter to compare with")
        elif values.get(name) != theirs.value:
            problems.append(
                f"{name} = {values.get(name)!r} but {CARTESIAN} uses {theirs.value!r} "
                f"(add --param {name}={theirs.value})"
            )
    theirs = crisp.get("filter.target_pose")
    if theirs is not None and values.get("target_filter") != theirs.value:
        warnings.append(
            f"target_filter = {values.get('target_filter')!r} but {CARTESIAN}'s filter.target_pose "
            f"is {theirs.value!r}: the two filter the same target differently"
        )
    # Both resolve a relative name in /lbr, so the same string is the same topic.
    theirs = crisp.get("topics.target_pose")
    if theirs is not None and values.get("target_topic") != theirs.value:
        problems.append(
            f"target_topic = {values.get('target_topic')!r} but {CARTESIAN} reads "
            f"{theirs.value!r}: the IK would chase a different target than crisp"
        )
    return problems, warnings


def _axes(flags) -> str:
    return ", ".join(f"A{j + 1}" for j, bad in enumerate(flags) if bad)


def limit_changes(values: dict[str, Any]) -> list[str]:
    """What in these (final) values loosens the guard on Sunrise's spring torque beyond the spec.

    Legal, and never to be done in passing: `set` refuses these for lagged and ik_target unless
    the change has had the review docs/commissioning_protocol.md requires.
    """
    out = []
    if "sunrise_joint_stiffness" in values:
        k = _per_joint(values["sunrise_joint_stiffness"])
        if k != list(DEFAULT_STIFFNESS):
            out.append(
                f"sunrise_joint_stiffness = {_fmt(k)} is not the stock LBRServer's 200 on every "
                "joint. It must equal the stiffness of the Sunrise app actually running (FRI does "
                "not report it): a value below the app's widens the gap, so the spring's torque "
                "exceeds max_spring_torque_nm; a stiffer app needs the review "
                "docs/commissioning_protocol.md requires for stiffness changes"
            )
    if "max_spring_torque_nm" in values:
        tau = _per_joint(values["max_spring_torque_nm"])
        if any(a > b for a, b in zip(tau, DEFAULT_SPRING_TORQUE)):
            out.append(
                f"max_spring_torque_nm = {_fmt(tau)} exceeds the spec's "
                f"{_fmt(list(DEFAULT_SPRING_TORQUE))} on "
                f"{_axes(a > b for a, b in zip(tau, DEFAULT_SPRING_TORQUE))}: the gap clamp is "
                "the only guard on Sunrise's spring torque under ik_target"
            )
    gap = spring_gap(values)
    wider = [g > d * (1 + 1e-12) for g, d in zip(gap or (), DEFAULT_GAP)]
    if any(wider):
        out.append(
            f"the gap {_fmt([round(g, 4) for g in gap])} rad is wider than the spec's "
            f"{_fmt(list(DEFAULT_GAP))} on {_axes(wider)}"
        )
    if "max_joint_step_rad" in values:
        step = _per_joint(values["max_joint_step_rad"])
        if any(x > DEFAULT_MAX_STEP for x in step):
            out.append(
                f"max_joint_step_rad = {_fmt(step)} exceeds the spec's {DEFAULT_MAX_STEP} "
                f"(2 rad/s at 500 Hz) on {_axes(x > DEFAULT_MAX_STEP for x in step)}"
            )
    return out


def _print_limits(values: dict[str, Any]) -> None:
    """The bound the reviewed change sets: per joint, the gap and the spring torque it allows."""
    gap = spring_gap(values)
    if gap is None:
        return
    k = _per_joint(values["sunrise_joint_stiffness"])
    print(
        f"LIMIT CHANGE (--limit-change-reviewed): |q_out - q_meas| <= "
        f"{_fmt([round(g, 4) for g in gap])} rad; at sunrise_joint_stiffness {_fmt(k)} Nm/rad "
        f"Sunrise's spring may add up to {_fmt([round(g * s, 2) for g, s in zip(gap, k)])} Nm; "
        f"max_joint_step_rad {_fmt(_per_joint(values.get('max_joint_step_rad', DEFAULT_MAX_STEP)))}"
    )


def parse_state(data: list[float]) -> dict[str, Any] | None:
    """The controller's ~/state message; the per-joint parts only if it has the listed layout."""
    if len(data) < 2:
        return None
    index = int(data[0]) if math.isfinite(data[0]) else -1
    out: dict[str, Any] = {
        "mode": MODES[index] if 0 <= index < len(MODES) else f"unknown ({data[0]})",
        "stale": data[1] != 0,
        "size": len(data),
    }
    if len(data) == STATE_SIZE:
        seven = [list(data[2 + k * JOINTS : 2 + (k + 1) * JOINTS]) for k in range(5)]
        out |= {
            "q_meas": seven[0],
            "q_ik": seven[1],
            "q_out": seven[2],
            "gap_clamped": [f"A{j + 1}" for j, v in enumerate(seven[3]) if v != 0],
            "step_clamped": [f"A{j + 1}" for j, v in enumerate(seven[4]) if v != 0],
        }
    return out


def _fmt(value: Any) -> str:
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(f"{v:g}" if isinstance(v, float) else str(v) for v in value) + "]"
    return f"{value:g}" if isinstance(value, float) else str(value)


def _print_gap(values: dict[str, Any]) -> None:
    gap = spring_gap(values)
    if gap is not None:
        print(
            f"spring gap |q_out - q_meas| <= {_fmt([round(g, 4) for g in gap])} rad "
            f"= {_fmt([round(math.degrees(g), 2) for g in gap])} deg (unused by measured)"
        )


# -- commands -------------------------------------------------------------------------------


def cmd_status(cell: Cell) -> int:
    states = cell.controller_states()
    print(f"cell: {cell_state(states)}")
    if PASSTHROUGH not in states:
        print(f"{PASSTHROUGH}: NOT LOADED")
        return 1
    print(f"{PASSTHROUGH}: {states[PASSTHROUGH]}")
    live = cell.get_parameters(PASSTHROUGH, PARAMETERS)
    if live is None:
        print(f"{PASSTHROUGH}'s parameter services did not answer")
        return 1
    if "position_source" not in live:
        print(
            "position_source is not declared: this controller predates "
            "feat/drake-joint-impedance and always writes the measured position"
        )
        return 0
    width = max(map(len, PARAMETERS))
    for name in PARAMETERS:
        shown = _fmt(live[name].value) if name in live else "(not declared)"
        print(f"  {name:<{width}}  {shown}")
    values = {n: p.value for n, p in live.items()}
    _print_gap(values)

    if states[PASSTHROUGH] != "active":
        print("not running: these take effect at the next arm (REST -> ARMED)")
        return 0
    data = cell.read_state(1.0)
    running = parse_state(data) if data is not None else None
    if running is None:
        print(f"running mode unknown: nothing on {NS}/{PASSTHROUGH}/state within 1 s")
        return 0
    if "gap_clamped" in running:
        clamps = (
            f"gap-clamped {running['gap_clamped'] or 'none'}, "
            f"step-clamped {running['step_clamped'] or 'none'}"
        )
    else:
        clamps = f"clamps not shown: {running['size']} values, the listed layout has {STATE_SIZE}"
    print(f"running (~/state): {running['mode']}, stale {int(running['stale'])}, {clamps}")
    if running["mode"] != values["position_source"]:
        print(
            f"the parameter ({values['position_source']}) differs from the running mode: "
            "it takes effect at the next arm"
        )
    return 0


def cmd_set(
    cell: Cell, mode: str, assignments: list[str], limit_change_reviewed: bool = False
) -> int:
    if mode not in MODES:
        print(f"REFUSED: {mode!r} is not one of {', '.join(MODES)}", file=sys.stderr)
        return 1
    try:
        texts = parse_assignments(assignments)
    except ValueError as why:
        print(f"REFUSED: {why}", file=sys.stderr)
        return 1

    states = cell.controller_states()
    problem = rest_problem(states)
    if problem:
        print(f"REFUSED: {problem}. Nothing was set.", file=sys.stderr)
        return 1

    live = cell.get_parameters(PASSTHROUGH, PARAMETERS)
    if live is None:
        print(f"REFUSED: {PASSTHROUGH}'s parameter services did not answer", file=sys.stderr)
        return 1
    try:
        request = {"position_source": coerce("position_source", mode, live.get("position_source"))}
        request |= {name: coerce(name, text, live.get(name)) for name, text in texts.items()}
    except ValueError as why:
        print(f"REFUSED: {why}. Nothing was set.", file=sys.stderr)
        return 1

    final = {n: p.value for n, p in live.items()} | {n: p.value for n, p in request.items()}
    problems = validate(final)
    warnings: list[str] = []
    limits = limit_changes(final)
    if limits and mode == "measured":
        # Unused by measured, so `set measured` always works; the next other mode is refused.
        warnings += [
            f"{why} (unused by measured; refused for lagged and ik_target)" for why in limits
        ]
    elif limits and not limit_change_reviewed:
        problems += [
            f"{why}. A limit change, not a tuning step: only after the review "
            "docs/commissioning_protocol.md requires, with --limit-change-reviewed"
            for why in limits
        ]
    if mode == "ik_target":
        crisp = cell.get_parameters(CARTESIAN, CRISP_PARAMETERS)
        more, also = ik_target_checks(final, crisp)
        problems += more
        warnings += also
    if problems:
        for why in problems:
            print(f"REFUSED: {why}", file=sys.stderr)
        print("Nothing was set.", file=sys.stderr)
        return 1
    if limits and mode != "measured":
        for why in limits:
            print(f"LIMIT CHANGE: {why}")
        _print_limits(final)

    why = cell.set_parameters(PASSTHROUGH, request)
    if why is not None:
        print(
            f"FAILED: {PASSTHROUGH} refused the change ({why or 'no reason given'}); "
            "an atomic set changes nothing on refusal",
            file=sys.stderr,
        )
        return 1
    print(
        f"set on {NS}/{PASSTHROUGH}: "
        + ", ".join(f"{n} = {_fmt(p.value)}" for n, p in request.items())
    )

    back = cell.get_parameters(PASSTHROUGH, tuple(request)) or {}
    wrong = [n for n, p in request.items() if n not in back or back[n].value != p.value]
    if wrong:
        print(
            "FAILED: read back differs for "
            + ", ".join(f"{n} ({_fmt(back[n].value) if n in back else 'missing'})" for n in wrong),
            file=sys.stderr,
        )
        return 1
    print("read back: identical")
    for why in warnings:
        print(f"WARNING: {why}")
    _print_gap(final)

    # Something could have armed the cell between the REST check and the set; then whether the
    # new values made that activation is a race. Say so instead of claiming the next arm.
    after = rest_problem(cell.controller_states())
    if after:
        print(
            f"WARNING: the cell left REST during the change ({after}); the running mode is "
            "uncertain: check `position-source status`",
            file=sys.stderr,
        )
        return 1
    print(
        f"position_source {mode}: takes effect at the next arm (REST -> ARMED), "
        "which logs the mode it activated with"
    )
    return 0


# -- ROS ------------------------------------------------------------------------------------


class RosCell:
    """The Cell methods over the controller manager's and the controllers' services."""

    def __init__(self, namespace: str = NS, timeout_s: float = 5.0) -> None:
        # Imported here so the decision logic above imports without a ROS environment.
        import rclpy
        from rcl_interfaces.msg import ParameterType

        self.rclpy = rclpy
        self.ns = namespace
        self.timeout_s = timeout_s
        self.node = rclpy.create_node("position_source")
        # ParameterType constant -> (type name, ParameterValue field)
        self.types = {
            ParameterType.PARAMETER_NOT_SET: ("not_set", None),
            ParameterType.PARAMETER_BOOL: ("bool", "bool_value"),
            ParameterType.PARAMETER_INTEGER: ("integer", "integer_value"),
            ParameterType.PARAMETER_DOUBLE: ("double", "double_value"),
            ParameterType.PARAMETER_STRING: ("string", "string_value"),
            ParameterType.PARAMETER_BYTE_ARRAY: ("byte_array", "byte_array_value"),
            ParameterType.PARAMETER_BOOL_ARRAY: ("bool_array", "bool_array_value"),
            ParameterType.PARAMETER_INTEGER_ARRAY: ("integer_array", "integer_array_value"),
            ParameterType.PARAMETER_DOUBLE_ARRAY: ("double_array", "double_array_value"),
            ParameterType.PARAMETER_STRING_ARRAY: ("string_array", "string_array_value"),
        }

    def _call(self, srv: Any, name: str, request: Any, *, required: bool = True) -> Any:
        client = self.node.create_client(srv, name)
        try:
            if not client.wait_for_service(timeout_sec=self.timeout_s):
                if required:
                    raise SystemExit(f"service {name} is not up -- is the stack running?")
                return None
            future = client.call_async(request)
            self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout_s)
            if not future.done():
                raise SystemExit(f"service {name} did not answer in {self.timeout_s} s")
            return future.result()
        finally:
            self.node.destroy_client(client)

    def controller_states(self) -> dict[str, str]:
        from controller_manager_msgs.srv import ListControllers

        result = self._call(
            ListControllers,
            f"{self.ns}/controller_manager/list_controllers",
            ListControllers.Request(),
        )
        return {c.name: c.state for c in result.controller}

    def get_parameters(self, node: str, names: tuple[str, ...]) -> dict[str, Param] | None:
        from rcl_interfaces.srv import GetParameters, ListParameters

        # rclcpp's get_parameters answers NOTHING if any name is undeclared, so ask which are.
        listed = self._call(
            ListParameters,
            f"{self.ns}/{node}/list_parameters",
            ListParameters.Request(),
            required=False,
        )
        if listed is None:
            return None
        known = set(listed.result.names)
        declared = [n for n in names if n in known]
        if not declared:
            return {}
        got = self._call(
            GetParameters, f"{self.ns}/{node}/get_parameters", GetParameters.Request(names=declared)
        )
        out = {}
        for name, value in zip(declared, got.values):
            kind, field = self.types[value.type]
            raw = getattr(value, field) if field else None
            out[name] = Param(kind, list(raw) if kind.endswith("_array") else raw)
        return out

    def set_parameters(self, node: str, params: dict[str, Param]) -> str | None:
        from rcl_interfaces.msg import Parameter, ParameterValue
        from rcl_interfaces.srv import SetParametersAtomically

        by_name = {kind: (code, field) for code, (kind, field) in self.types.items()}
        messages = []
        for name, param in params.items():
            code, field = by_name[param.type]
            value = ParameterValue(type=code)
            setattr(value, field, param.value)
            messages.append(Parameter(name=name, value=value))
        result = self._call(
            SetParametersAtomically,
            f"{self.ns}/{node}/set_parameters_atomically",
            SetParametersAtomically.Request(parameters=messages),
        )
        return None if result.result.successful else result.result.reason

    def read_state(self, timeout_s: float) -> list[float] | None:
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Float64MultiArray

        got: list[list[float]] = []
        # Best effort matches a realtime publisher of either reliability.
        sub = self.node.create_subscription(
            Float64MultiArray,
            f"{self.ns}/{PASSTHROUGH}/state",
            lambda m: got.append(list(m.data)),
            qos_profile_sensor_data,
        )
        deadline = time.monotonic() + timeout_s
        while not got and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(sub)
        return got[-1] if got else None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--namespace", default=NS)
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status", help="the parameters, the controller's state and its running mode")
    st = sub.add_parser(
        "set", help="at REST only: set position_source (and --param) for the next arm"
    )
    st.add_argument("mode", choices=MODES)
    st.add_argument(
        "--param",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="another position_source parameter, e.g. lag_time_constant_s=0.01 or "
        "'max_spring_torque_nm=[5,5,5,4,3,3,2]' (quoted: a shell globs brackets); repeatable",
    )
    st.add_argument(
        "--limit-change-reviewed",
        action="store_true",
        help="allow values looser than the spec's spring-torque guard (a wider gap, a stiffness "
        "other than 200, more max_spring_torque_nm or max_joint_step_rad): only after the review "
        "docs/commissioning_protocol.md requires. Prints the resulting gap and torque first",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    import rclpy

    rclpy.init()
    try:
        cell = RosCell(args.namespace)
        if args.cmd == "status":
            return cmd_status(cell)
        return cmd_set(cell, args.mode, args.param, args.limit_change_reviewed)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
