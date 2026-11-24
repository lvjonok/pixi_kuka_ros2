"""The iiwa cell as this repo's stack runs it: names, REST and ARMED, the switch, joint limits."""

import threading
import time
import xml.etree.ElementTree as ET
from collections.abc import Iterator, Sequence
from contextlib import contextmanager
from typing import Any

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.executors import ExternalShutdownException, ShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

NS = "/lbr"
JOINTS = tuple(f"lbr_A{i}" for i in range(1, 8))
BASE_FRAME = "lbr_link_0"
HOME_DEGREES = (0.0, 30.0, 0.0, -75.0, 0.0, 75.0, 0.0)

PASSTHROUGH = "fri_position_passthrough_controller"
TRAJECTORY = "joint_trajectory_controller"
ZERO_EFFORT = "zero_effort_controller"
CARTESIAN = "cartesian_impedance_controller"
TORQUE_CONTROLLERS = (CARTESIAN, "joint_impedance_controller")

# REST: the trajectory controller holds a fixed position command and Sunrise's joint spring holds
# the arm on it. ARMED: the passthrough's position command follows the arm and a torque controller
# holds it. The passthrough with zero effort holds nothing: the arm sags.
REST = (TRAJECTORY, ZERO_EFFORT)
ARMED = (PASSTHROUGH, CARTESIAN)


@contextmanager
def spinning(node: Node) -> Iterator[None]:
    """Spin the node in a thread for the block.

    The thread ends before the block is left: rclpy aborts a process that exits while one spins.
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin() -> None:
        try:
            executor.spin()
        except (ExternalShutdownException, ShutdownException):
            pass

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        yield
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)


def call(node: Node, service: Any, name: str, request: Any, timeout_s: float = 5.0) -> Any:
    """The answer of one service call. Another thread spins the node."""
    client = node.create_client(service, name)
    try:
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"service {name} is not up: is the stack running?")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError(f"service {name} did not answer in {timeout_s:g} s")
            time.sleep(0.01)
        return future.result()
    finally:
        node.destroy_client(client)


def controller_states(node: Node) -> dict[str, str]:
    answer = call(
        node,
        ListControllers,
        f"{NS}/controller_manager/list_controllers",
        ListControllers.Request(),
    )
    return {controller.name: controller.state for controller in answer.controller}


def cell_state(states: dict[str, str]) -> str:
    """REST, ARMED with its torque controllers, or what else holds the commands."""
    active = {name for name, state in states.items() if state == "active"}
    if set(REST) <= active and PASSTHROUGH not in active:
        return "REST"
    overlays = sorted(set(TORQUE_CONTROLLERS) & active)
    if PASSTHROUGH in active and overlays:
        return f"ARMED ({', '.join(overlays)})"
    owners = sorted({PASSTHROUGH, *REST, *TORQUE_CONTROLLERS} & active)
    return f"neither REST nor ARMED (active: {', '.join(owners) or 'none of the command owners'})"


def switch(node: Node, *, activate: Sequence[str], deactivate: Sequence[str]) -> None:
    """One STRICT switch: all of it happens or none of it. Raises when it is refused."""
    request = SwitchController.Request(
        activate_controllers=list(activate),
        deactivate_controllers=list(deactivate),
        strictness=SwitchController.Request.STRICT,
        activate_asap=True,
        timeout=rclpy.duration.Duration(seconds=10).to_msg(),
    )
    name = f"{NS}/controller_manager/switch_controller"
    if not call(node, SwitchController, name, request, timeout_s=10.0).ok:
        raise RuntimeError(
            f"STRICT switch failed: activate {list(activate)}, deactivate {list(deactivate)}"
        )


def joint_limits(node: Node) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(lower, upper, velocity) of JOINTS, rad and rad/s, from the stack's robot_description."""
    heard: list[str] = []
    subscription = node.create_subscription(
        String,
        f"{NS}/robot_description",
        lambda message: heard.append(message.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    deadline = time.monotonic() + 10.0
    while not heard and time.monotonic() < deadline:
        time.sleep(0.05)
    node.destroy_subscription(subscription)
    if not heard:
        raise RuntimeError(f"no {NS}/robot_description within 10 s: is the stack up?")
    joints = ET.fromstring(heard[0]).findall("joint")
    limits = {joint.get("name"): joint.find("limit") for joint in joints}
    return tuple(
        np.array([float(limits[joint].get(key)) for joint in JOINTS])
        for key in ("lower", "upper", "velocity")
    )
