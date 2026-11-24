"""Show, and at REST set, what fri_position_passthrough_controller writes into FRI's position field.

    pixi run -e jazzy position-source status
    pixi run -e jazzy position-source set lagged
    pixi run -e jazzy position-source set ik_target --param target_timeout_s=0.2
    pixi run -e jazzy position-source set measured

The controller reads its parameters when it is activated, so `set` works at REST only and the
change holds from the next arm. For lagged and ik_target a value that loosens the bound on
Sunrise's spring torque is refused without --limit-change-reviewed. Moves nothing and switches
no controller. docs/position_source.md describes the modes.
"""

import argparse
import math
import sys
import time
from typing import Any

import rclpy
from lbr_cell import (
    CARTESIAN,
    JOINTS,
    NS,
    PASSTHROUGH,
    REST,
    call,
    cell_state,
    controller_states,
    spinning,
)
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

MODES = ("measured", "lagged", "ik_target")
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
CRISP_PARAMETERS = ("end_effector_frame", "base_frame", "topics.target_pose", "filter.target_pose")

# The bound on Sunrise's spring torque that `set` allows for lagged and ik_target without
# --limit-change-reviewed.
STIFFNESS = [200.0] * len(JOINTS)
SPRING_TORQUE = [10.0, 10.0, 10.0, 8.0, 5.0, 5.0, 3.0]
GAP = [torque / stiffness for torque, stiffness in zip(SPRING_TORQUE, STIFFNESS)]
MAX_STEP = 0.004


def get_parameters(node: Node, controller: str, names: tuple[str, ...]) -> dict[str, Any]:
    answer = call(
        node,
        GetParameters,
        f"{NS}/{controller}/get_parameters",
        GetParameters.Request(names=list(names)),
    )
    # rclcpp answers with no value at all when one of the names is not declared.
    if len(answer.values) != len(names):
        raise RuntimeError(
            f"{controller} does not declare {', '.join(names)}: is this repo's stack running?"
        )
    return {name: parameter_value_to_python(value) for name, value in zip(names, answer.values)}


def source(node: Node) -> str:
    return get_parameters(node, PASSTHROUGH, ("position_source",))["position_source"]


def running(node: Node, timeout_s: float) -> dict[str, Any] | None:
    """The passthrough's newest ~/state: its mode, the stale flag and the joints each clamp cut."""
    heard: list[list[float]] = []
    subscription = node.create_subscription(
        Float64MultiArray,
        f"{NS}/{PASSTHROUGH}/state",
        lambda message: heard.append(list(message.data)),
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + timeout_s
    while not heard and time.monotonic() < deadline:
        time.sleep(0.01)
    node.destroy_subscription(subscription)
    if not heard:
        return None
    # [mode, stale, q_meas[7], q_ik[7], q_out[7], gap_clamped[7], step_clamped[7]]
    data, n = heard[-1], len(JOINTS)
    return {
        "mode": MODES[int(data[0])],
        "stale": data[1] != 0,
        "gap_clamped": _axes(data[2 + 3 * n : 2 + 4 * n]),
        "step_clamped": _axes(data[2 + 4 * n : 2 + 5 * n]),
    }


def rest_problem(states: dict[str, str]) -> str | None:
    """Why the cell is not at REST, or None."""
    if PASSTHROUGH not in states:
        return f"{PASSTHROUGH} is not loaded: is this repo's stack running?"
    if states[PASSTHROUGH] == "active":
        return f"{PASSTHROUGH} is active (the cell is {cell_state(states)}): REST it first"
    missing = [controller for controller in REST if states.get(controller) != "active"]
    if missing:
        return f"not at REST: {', '.join(missing)} not active (the cell is {cell_state(states)})"
    return None


def parse_assignments(items: list[str]) -> dict[str, str]:
    out: dict[str, str] = {}
    for item in items:
        name, separator, text = item.partition("=")
        name = name.strip()
        if not separator or not name:
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


def coerce(name: str, text: str, live: Any) -> Any:
    """``text`` as the type the controller holds ``name`` in: a string, a number, or one per joint.

    One number for a per-joint parameter goes to every joint.
    """
    if isinstance(live, str):
        return text
    try:
        values = [float(part) for part in text.strip("[]").replace(",", " ").split()]
    except ValueError:
        raise ValueError(f"{name}: {text!r} is not a number") from None
    if not all(map(math.isfinite, values)):
        raise ValueError(f"{name}: {text!r} is not finite")
    if isinstance(live, float):
        if len(values) != 1:
            raise ValueError(f"{name}: one number, not {len(values)}")
        return values[0]
    if len(values) == 1:
        values = values * len(live)
    if len(values) != len(live):
        raise ValueError(f"{name}: {len(values)} values, the controller holds {len(live)}")
    return values


def validate(values: dict[str, Any]) -> list[str]:
    """What is wrong with these values. The controller fails its activation on them."""
    checks = (
        ("lag_time_constant_s", lambda v: v >= 0, "must be >= 0"),
        ("target_filter", lambda v: 0 <= v < 1, "must be in [0, 1); 1 freezes the target"),
        ("target_timeout_s", lambda v: v > 0, "must be > 0"),
        ("ik_damping", lambda v: v >= 0, "must be >= 0"),
        ("nullspace_gain", lambda v: 0 <= v <= 1, "must be in [0, 1] (a per-update fraction)"),
        ("joint_limit_margin_rad", lambda v: v >= 0, "must be >= 0"),
        ("max_joint_step_rad", lambda v: all(x > 0 for x in v), "must be > 0"),
        ("sunrise_joint_stiffness", lambda v: all(x > 0 for x in v), "must be > 0"),
        ("max_spring_torque_nm", lambda v: all(x >= 0 for x in v), "must be >= 0"),
        ("target_topic", lambda v: v.strip() != "", "must not be empty"),
        ("end_effector_frame", lambda v: v.strip() != "", "must not be empty"),
        ("base_frame", lambda v: v.strip() != "", "must not be empty"),
    )
    return [f"{name} = {values[name]!r}: {why}" for name, ok, why in checks if not ok(values[name])]


def spring_gap(values: dict[str, Any]) -> list[float]:
    """Per joint, the largest |q_out - q_meas| the clamp allows, rad."""
    pairs = zip(values["max_spring_torque_nm"], values["sunrise_joint_stiffness"])
    return [torque / stiffness if stiffness > 0 else math.inf for torque, stiffness in pairs]


def ik_target_problems(values: dict[str, Any], crisp: dict[str, Any]) -> list[str]:
    """Where ik_target would not solve for the frame and the target crisp is driving."""
    pairs = (
        ("end_effector_frame", "end_effector_frame"),
        ("base_frame", "base_frame"),
        ("target_topic", "topics.target_pose"),
    )
    return [
        f"{ours} = {values[ours]!r} but {CARTESIAN}'s {theirs} is {crisp[theirs]!r} "
        f"(add --param {ours}={crisp[theirs]})"
        for ours, theirs in pairs
        if values[ours] != crisp[theirs]
    ]


def limit_changes(values: dict[str, Any]) -> list[str]:
    """What in these values loosens the bound on Sunrise's spring torque."""
    out = []
    stiffness = values["sunrise_joint_stiffness"]
    if stiffness != STIFFNESS:
        out.append(
            f"sunrise_joint_stiffness = {_fmt(stiffness)} is not 200 on every joint. It must be "
            "the stiffness of the Sunrise app that runs (FRI does not report it): a value below "
            "the app's widens the gap, so the spring's torque exceeds max_spring_torque_nm"
        )
    torque = values["max_spring_torque_nm"]
    if any(a > b for a, b in zip(torque, SPRING_TORQUE)):
        out.append(
            f"max_spring_torque_nm = {_fmt(torque)} exceeds {_fmt(SPRING_TORQUE)} on "
            f"{_axes(a > b for a, b in zip(torque, SPRING_TORQUE))}: the gap clamp is the only "
            "bound on Sunrise's spring torque"
        )
    gap = spring_gap(values)
    wider = [g > d * (1 + 1e-12) for g, d in zip(gap, GAP)]
    if any(wider):
        out.append(
            f"the gap {_fmt([round(g, 4) for g in gap])} rad is wider than {_fmt(GAP)} on "
            f"{_axes(wider)}"
        )
    step = values["max_joint_step_rad"]
    if any(x > MAX_STEP for x in step):
        out.append(
            f"max_joint_step_rad = {_fmt(step)} exceeds {MAX_STEP} (2 rad/s at 500 Hz) on "
            f"{_axes(x > MAX_STEP for x in step)}"
        )
    return out


def _axes(flags: Any) -> str:
    return ", ".join(f"A{j + 1}" for j, flag in enumerate(flags) if flag)


def _fmt(value: Any) -> str:
    if isinstance(value, list):
        return "[" + ", ".join(f"{v:g}" for v in value) + "]"
    return f"{value:g}" if isinstance(value, float) else str(value)


def _gap_line(values: dict[str, Any]) -> str:
    gap = spring_gap(values)
    return (
        f"spring gap |q_out - q_meas| <= {_fmt([round(g, 4) for g in gap])} rad "
        f"= {_fmt([round(math.degrees(g), 2) for g in gap])} deg (unused by measured)"
    )


def status(node: Node) -> int:
    states = controller_states(node)
    print(f"cell: {cell_state(states)}")
    if PASSTHROUGH not in states:
        print(f"{PASSTHROUGH}: NOT LOADED")
        return 1
    print(f"{PASSTHROUGH}: {states[PASSTHROUGH]}")
    values = get_parameters(node, PASSTHROUGH, PARAMETERS)
    width = max(map(len, PARAMETERS))
    for name in PARAMETERS:
        print(f"  {name:<{width}}  {_fmt(values[name])}")
    print(_gap_line(values))

    if states[PASSTHROUGH] != "active":
        print("not running: these take effect at the next arm (REST -> ARMED)")
        return 0
    state = running(node, 1.0)
    if state is None:
        print(f"running mode unknown: nothing on {NS}/{PASSTHROUGH}/state within 1 s")
        return 0
    print(
        f"running (~/state): {state['mode']}, stale {int(state['stale'])}, "
        f"gap-clamped {state['gap_clamped'] or 'none'}, "
        f"step-clamped {state['step_clamped'] or 'none'}"
    )
    if state["mode"] != values["position_source"]:
        print(
            f"the parameter ({values['position_source']}) differs from the running mode: "
            "it takes effect at the next arm"
        )
    return 0


def set_source(node: Node, mode: str, assignments: list[str], reviewed: bool) -> int:
    problem = rest_problem(controller_states(node))
    if problem:
        print(f"REFUSED: {problem}. Nothing was set.", file=sys.stderr)
        return 1
    live = get_parameters(node, PASSTHROUGH, PARAMETERS)
    try:
        texts = parse_assignments(assignments)
        request = {"position_source": mode}
        request |= {name: coerce(name, text, live[name]) for name, text in texts.items()}
    except ValueError as why:
        print(f"REFUSED: {why}. Nothing was set.", file=sys.stderr)
        return 1

    final = live | request
    problems = validate(final)
    warnings: list[str] = []
    limits = limit_changes(final)
    if mode == "measured":
        warnings += [
            f"{why} (unused by measured; refused for lagged and ik_target)" for why in limits
        ]
    elif not reviewed:
        problems += [
            f"{why}. A limit change, not a tuning step: only after the review "
            "docs/commissioning_protocol.md asks for, with --limit-change-reviewed"
            for why in limits
        ]
    if mode == "ik_target":
        crisp = get_parameters(node, CARTESIAN, CRISP_PARAMETERS)
        problems += ik_target_problems(final, crisp)
        if final["target_filter"] != crisp["filter.target_pose"]:
            warnings.append(
                f"target_filter = {final['target_filter']!r} but {CARTESIAN}'s filter.target_pose "
                f"is {crisp['filter.target_pose']!r}: the two filter the same target differently"
            )
    if problems:
        for why in problems:
            print(f"REFUSED: {why}", file=sys.stderr)
        print("Nothing was set.", file=sys.stderr)
        return 1
    if mode != "measured" and limits:
        gap = spring_gap(final)
        stiffness = final["sunrise_joint_stiffness"]
        for why in limits:
            print(f"LIMIT CHANGE: {why}")
        print(
            f"LIMIT CHANGE (--limit-change-reviewed): |q_out - q_meas| <= "
            f"{_fmt([round(g, 4) for g in gap])} rad; at sunrise_joint_stiffness "
            f"{_fmt(stiffness)} Nm/rad Sunrise's spring may add up to "
            f"{_fmt([round(g * k, 2) for g, k in zip(gap, stiffness)])} Nm; "
            f"max_joint_step_rad {_fmt(final['max_joint_step_rad'])}"
        )

    answer = call(
        node,
        SetParametersAtomically,
        f"{NS}/{PASSTHROUGH}/set_parameters_atomically",
        SetParametersAtomically.Request(
            parameters=[Parameter(n, value=v).to_parameter_msg() for n, v in request.items()]
        ),
    )
    if not answer.result.successful:
        print(
            f"FAILED: {PASSTHROUGH} refused the change "
            f"({answer.result.reason or 'no reason given'}); nothing was set",
            file=sys.stderr,
        )
        return 1
    print(
        f"set on {NS}/{PASSTHROUGH}: " + ", ".join(f"{n} = {_fmt(v)}" for n, v in request.items())
    )
    back = get_parameters(node, PASSTHROUGH, tuple(request))
    wrong = [name for name, value in request.items() if back[name] != value]
    if wrong:
        print(
            "FAILED: read back differs for "
            + ", ".join(f"{name} ({_fmt(back[name])})" for name in wrong),
            file=sys.stderr,
        )
        return 1
    print("read back: identical")
    for why in warnings:
        print(f"WARNING: {why}")
    print(_gap_line(final))

    # The cell may have been armed between the check above and the set: then the mode that
    # activation took is not known.
    after = rest_problem(controller_states(node))
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


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("status", help="the parameters, the controller's state, the running mode")
    setter = commands.add_parser(
        "set", help="at REST only: the mode, and --param, for the next arm"
    )
    setter.add_argument("mode", choices=MODES)
    setter.add_argument(
        "--param",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="another position_source parameter, e.g. lag_time_constant_s=0.01 or "
        "'max_spring_torque_nm=[5,5,5,4,3,3,2]' (quoted: a shell globs brackets); repeatable",
    )
    setter.add_argument(
        "--limit-change-reviewed",
        action="store_true",
        help="allow values that loosen the bound on Sunrise's spring torque (a wider gap, a "
        "stiffness other than 200, more max_spring_torque_nm or max_joint_step_rad). Prints the "
        "resulting gap and torque first",
    )
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("position_source")
    try:
        with spinning(node):
            if args.command == "status":
                return status(node)
            return set_source(node, args.mode, args.param, args.limit_change_reviewed)
    except RuntimeError as why:
        print(why, file=sys.stderr)
        return 1
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
