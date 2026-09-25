"""Where, and how hard, the arm feels contact: a read-only viser page.

For a stack launched with `tool:=umi` (`pixi run -e jazzy hardware-umi`). It draws the arm from
its own robot_description and two things the controllers already publish:

* **The estimated contact wrench** (`/lbr/force_torque_broadcaster/wrench`), as an arrow at the
  frame named in its header -- `lbr_umi_tweezer_tip` with `controllers_umi.yaml`. It is KUKA's
  external joint torque (FRI `tau_ext`, measured minus the model with the declared load) mapped
  through the URDF Jacobian by lbr-stack's WrenchEstimator, so it is only as good as the load
  declared in Sunrise and only meaningful for contact AT that frame. The arrow is the force as
  published, rotated into the base by this page's own FK of the header frame on the live joint
  angles; the readout gives it in base and in tip axes.
* **The external torque per joint** (`/lbr/kuka_external_torque`, kuka_ext_torque_relay.py), as
  a sphere at each joint sized and coloured by |tau_ext| and an arrow along the joint axis
  (right-hand rule, signed). This is the raw signal the wrench is estimated from, and the one
  that still says something when contact is on a link rather than at the tip: the joints
  between the base and the contact carry it, the ones past it do not.

A bias is expected at rest (load-data error, friction, the estimator's damping), so **zero bias**
subtracts the current filtered reading in base axes -- the same axes lerobot_teleoperator_haply's
force step subtracts its bias in -- and the readout shows raw and biased side by side.

It publishes nothing, calls no controller_manager service and holds no controller: safe to run
beside a teleop, a recording or the camera gizmo.

    pixi run -e jazzy hardware-umi
    pixi run -e jazzy contact-view                     # prints a LAN link per address
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

NS = "/lbr"
BASE_FRAME = "lbr_link_0"
JOINT_NAMES = tuple(f"lbr_A{i + 1}" for i in range(7))
WRENCH_TOPIC = f"{NS}/force_torque_broadcaster/wrench"
TAU_EXT_TOPIC = f"{NS}/kuka_external_torque"

GREY = (150, 150, 150)
PLOT_SECONDS = 10.0


def _lan_links(port: int) -> list[str]:
    """One link per address this host has, so the page opens from the laptop."""
    try:
        addresses = subprocess.run(
            ["hostname", "-I"], capture_output=True, text=True, check=False
        ).stdout.split()
    except OSError:
        addresses = []
    return [f"http://{a}:{port}" for a in addresses if ":" not in a] or [
        f"http://localhost:{port}"
    ]


def _resolve_package_uri(fname: str) -> str:
    if not fname.startswith("package://"):
        return fname
    package, _, rest = fname[len("package://") :].partition("/")
    return str(Path(get_package_share_directory(package)) / rest)


def _heat(fraction: float) -> tuple[int, int, int]:
    """Green at 0, yellow at 0.5, red at 1 and beyond."""
    f = float(np.clip(fraction, 0.0, 1.0))
    if f < 0.5:
        return (int(510 * f), 200, 60)
    return (255, int(200 * (1.0 - 2.0 * (f - 0.5))), 60)


def configuration(model: pin.Model, names: list[str], positions: list[float]) -> np.ndarray:
    """A pinocchio q from a JointState: named joints set, everything else (fingers) neutral."""
    q = pin.neutral(model)
    for name, value in zip(names, positions, strict=False):
        if model.existJointName(name):
            q[model.joints[model.getJointId(name)].idx_q] = value
    return q


def wrench_in_base(
    model: pin.Model, data: pin.Data, q: np.ndarray, frame: str, force: np.ndarray, torque: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rotate a wrench expressed in `frame`'s axes into BASE_FRAME's.

    Returns:
        ``(force_base, torque_base, frame_origin_world)``. The torque is about the same point,
        only its axes change; the origin is in the model's world, where the page draws.
    """
    pin.framesForwardKinematics(model, data, q)
    world_base = data.oMf[model.getFrameId(BASE_FRAME)]
    world_frame = data.oMf[model.getFrameId(frame)]
    base_frame = world_base.actInv(world_frame)
    rotation = base_frame.rotation
    return rotation @ force, rotation @ torque, world_frame.translation.copy()


class ContactView(Node):
    """Subscribes, keeps the latest of each topic, and redraws the page on a timer."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Read the description, subscribe, build the page. Publishes nothing."""
        super().__init__("contact_view")
        self.args = args
        self.lock = threading.Lock()
        self.joints: tuple[list[str], list[float]] | None = None
        self.joints_at: float | None = None
        self.wrench: tuple[str, np.ndarray, np.ndarray] | None = None  # frame, F, T (header axes)
        self.wrench_at: float | None = None
        self.tau: dict[str, float] = {}
        self.tau_at: float | None = None

        self.filtered: np.ndarray | None = None  # [F_base, T_base], low-passed
        self.bias = np.zeros(6)
        self.history: deque[tuple[float, float, float]] = deque()  # (t, |F| raw, |F| biased)
        self.t0 = time.monotonic()

        self.urdf_xml = self._wait_for_description()
        self.model = pin.buildModelFromXML(self.urdf_xml)
        self.data = self.model.createData()
        if not self.model.existFrame(BASE_FRAME):
            raise SystemExit(f"robot_description has no {BASE_FRAME}")

        self.create_subscription(
            JointState, f"{NS}/joint_states", self._on_joints, qos_profile_sensor_data
        )
        self.create_subscription(
            WrenchStamped, WRENCH_TOPIC, self._on_wrench, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, TAU_EXT_TOPIC, self._on_tau, qos_profile_sensor_data
        )

        self._build_scene()
        self.create_timer(1.0 / args.rate, self._refresh)

    def _wait_for_description(self) -> str:
        """The description the stack is running, off robot_state_publisher's latched topic."""
        got: list[str] = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.create_subscription(
            String, f"{NS}/robot_description", lambda m: got.append(m.data), qos
        )
        deadline = time.monotonic() + 10.0
        while not got and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        if not got:
            raise SystemExit(f"no {NS}/robot_description within 10 s -- is the stack up?")
        return got[0]

    # -- subscriptions: store, nothing else ---------------------------------------------------

    def _on_joints(self, msg: JointState) -> None:
        with self.lock:
            self.joints = (list(msg.name), list(msg.position))
            self.joints_at = time.monotonic()

    def _on_wrench(self, msg: WrenchStamped) -> None:
        f, t = msg.wrench.force, msg.wrench.torque
        with self.lock:
            self.wrench = (
                msg.header.frame_id,
                np.array([f.x, f.y, f.z]),
                np.array([t.x, t.y, t.z]),
            )
            self.wrench_at = time.monotonic()

    def _on_tau(self, msg: JointState) -> None:
        with self.lock:
            self.tau = dict(zip(msg.name, msg.effort, strict=False))
            self.tau_at = time.monotonic()

    # -- the page ------------------------------------------------------------------------------

    def _build_scene(self) -> None:
        import viser
        import yourdfpy
        from viser import uplot
        from viser.extras import ViserUrdf

        self.server = viser.ViserServer(host=self.args.host, port=self.args.port)
        scene, gui = self.server.scene, self.server.gui
        scene.add_grid("/floor", width=2.0, height=2.0, cell_size=0.1)

        with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
            f.write(self.urdf_xml)
        urdf = yourdfpy.URDF.load(f.name, filename_handler=_resolve_package_uri)
        self.viser_urdf = ViserUrdf(self.server, urdf_or_path=urdf, root_node_name="/arm")
        self.actuated = list(self.viser_urdf.get_actuated_joint_names())

        with gui.add_folder("status"):
            self.gui_status = gui.add_text("topics", initial_value="waiting", disabled=True)
            self.gui_frame = gui.add_text("wrench frame", initial_value="", disabled=True)
        with gui.add_folder("contact force"):
            self.gui_scale = gui.add_slider(
                "arrow cm per N", min=0.1, max=5.0, step=0.1, initial_value=1.0
            )
            self.gui_full = gui.add_slider(
                "red at N", min=1.0, max=50.0, step=1.0, initial_value=self.args.full_scale_n
            )
            self.gui_tau = gui.add_number(
                "low-pass tau s", initial_value=self.args.tau_s, min=0.0, max=2.0, step=0.01
            )
            self.gui_biased = gui.add_checkbox("draw biased", initial_value=True)
            zero = gui.add_button("zero bias (at rest, no contact)")
            clear = gui.add_button("clear bias")
            self.gui_fbase = gui.add_text("F base N", initial_value="", disabled=True)
            self.gui_fbase_b = gui.add_text("F base - bias", initial_value="", disabled=True)
            self.gui_ftip = gui.add_text("F tip axes", initial_value="", disabled=True)
            self.gui_norm = gui.add_text("|F| raw / biased", initial_value="", disabled=True)
            self.gui_torque = gui.add_text("T base Nm", initial_value="", disabled=True)
            self.gui_bias = gui.add_text("bias", initial_value="0", disabled=True)
        with gui.add_folder("joint tau_ext"):
            self.gui_tau_full = gui.add_slider(
                "red at Nm", min=1.0, max=50.0, step=1.0, initial_value=self.args.full_scale_nm
            )
            self.gui_taus = gui.add_text("Nm A1..A7", initial_value="", disabled=True)

        empty = np.zeros(0)
        self.plot = gui.add_uplot(
            data=(empty, empty, empty),
            series=(
                uplot.Series(label="t s"),
                uplot.Series(label="|F| raw", stroke="#888888", width=1),
                uplot.Series(label="|F| biased", stroke="#d04020", width=2),
            ),
            title="|F| N, last 10 s",
            aspect=2.0,
        )

        zero.on_click(lambda _: self._zero_bias())
        clear.on_click(lambda _: self._set_bias(np.zeros(6)))

        for link in _lan_links(self.args.port):
            print(f"  contact view on {link}", flush=True)

    def _zero_bias(self) -> None:
        if self.filtered is not None:
            self._set_bias(self.filtered.copy())

    def _set_bias(self, bias: np.ndarray) -> None:
        self.bias = bias
        f = bias[:3]
        self.gui_bias.value = f"F {_fmt(f)} N  |{np.linalg.norm(f):.2f}|"

    def _stale(self, at: float | None, now: float) -> bool:
        return at is None or now - at > self.args.stale_s

    def _refresh(self) -> None:
        now = time.monotonic()
        with self.lock:
            joints, joints_at = self.joints, self.joints_at
            wrench, wrench_at = self.wrench, self.wrench_at
            tau, tau_at = dict(self.tau), self.tau_at

        stale = {
            "joint_states": self._stale(joints_at, now),
            "wrench": self._stale(wrench_at, now),
            "tau_ext": self._stale(tau_at, now),
        }
        self.gui_status.value = (
            "ok" if not any(stale.values())
            else "STALE: " + ", ".join(k for k, v in stale.items() if v) + f" (> {self.args.stale_s} s)"
        )
        if joints is None:
            return
        q = configuration(self.model, *joints)
        self.viser_urdf.update_cfg(
            np.array([q[self.model.joints[self.model.getJointId(n)].idx_q]
                      if self.model.existJointName(n) else 0.0 for n in self.actuated])
        )
        self._draw_wrench(q, wrench, stale["wrench"] or stale["joint_states"], now)
        self._draw_joints(q, tau, stale["tau_ext"] or stale["joint_states"])

    def _draw_wrench(
        self, q: np.ndarray, wrench: tuple[str, np.ndarray, np.ndarray] | None, stale: bool, now: float
    ) -> None:
        scene = self.server.scene
        if wrench is None:
            self.gui_frame.value = f"no {WRENCH_TOPIC} yet"
            return
        frame, force, torque = wrench
        if not self.model.existFrame(frame):
            # Loud: a wrench in a frame this description does not have cannot be rotated, and a
            # guess would draw a confident arrow in the wrong direction.
            self.gui_frame.value = f"REFUSED: '{frame}' is not in robot_description"
            scene.add_arrows("/force", np.zeros((1, 2, 3)), GREY, visible=False)
            return
        self.gui_frame.value = frame
        f_base, t_base, origin = wrench_in_base(self.model, self.data, q, frame, force, torque)
        sample = np.concatenate([f_base, t_base])
        dt = 1.0 / self.args.rate
        tau_s = float(self.gui_tau.value)
        alpha = 1.0 if tau_s <= 0.0 else dt / (tau_s + dt)
        self.filtered = sample if self.filtered is None else self.filtered + alpha * (sample - self.filtered)
        raw = self.filtered[:3]
        biased = raw - self.bias[:3]
        shown = biased if self.gui_biased.value else raw

        self.gui_fbase.value = f"{_fmt(raw)}"
        self.gui_fbase_b.value = f"{_fmt(biased)}"
        self.gui_ftip.value = f"{_fmt(force)} (unfiltered, as published)"
        self.gui_norm.value = f"{np.linalg.norm(raw):.2f} / {np.linalg.norm(biased):.2f} N"
        self.gui_torque.value = f"{_fmt(self.filtered[3:] - self.bias[3:])}"

        # The model's world is where the page draws; rotate the base-axes force back into it.
        pin.framesForwardKinematics(self.model, self.data, q)
        world_base = self.data.oMf[self.model.getFrameId(BASE_FRAME)].rotation
        vector = world_base @ shown * float(self.gui_scale.value) / 100.0
        magnitude = float(np.linalg.norm(shown))
        color = GREY if stale else _heat(magnitude / float(self.gui_full.value))
        if np.linalg.norm(vector) < 1e-4:
            vector = np.zeros(3)
        scene.add_arrows(
            "/force",
            np.array([[origin, origin + vector]]),
            color,
            shaft_radius=0.004,
            head_radius=0.01,
            head_length=0.02,
        )
        scene.add_icosphere("/force_point", radius=0.006, color=color, position=tuple(origin))

        t = now - self.t0
        self.history.append((t, float(np.linalg.norm(raw)), float(np.linalg.norm(biased))))
        while self.history and self.history[0][0] < t - PLOT_SECONDS:
            self.history.popleft()
        data = np.array(self.history).T
        self.plot.data = (data[0], data[1], data[2])

    def _draw_joints(self, q: np.ndarray, tau: dict[str, float], stale: bool) -> None:
        scene = self.server.scene
        pin.forwardKinematics(self.model, self.data, q)
        full = float(self.gui_tau_full.value)
        values = []
        for name in JOINT_NAMES:
            if not self.model.existJointName(name):
                continue
            joint_id = self.model.getJointId(name)
            placement = self.data.oMi[joint_id]
            value = float(tau.get(name, 0.0))
            values.append(value)
            # lbr joints turn about their local z.
            axis = placement.rotation[:, 2]
            color = GREY if stale else _heat(abs(value) / full)
            origin = placement.translation
            scene.add_icosphere(
                f"/tau/{name}/sphere",
                radius=0.015 + 0.03 * min(abs(value) / full, 1.5),
                color=color,
                opacity=0.6,
                position=tuple(origin),
            )
            tip = origin + axis * 0.1 * np.clip(value / full, -1.5, 1.5)
            scene.add_arrows(
                f"/tau/{name}/axis",
                np.array([[origin, tip]]),
                color,
                shaft_radius=0.003,
                head_radius=0.008,
                head_length=0.015,
                visible=abs(value) > 0.02 * full,
            )
        self.gui_taus.value = " ".join(f"{v:+.1f}" for v in values)


def _fmt(v: np.ndarray) -> str:
    return "[" + " ".join(f"{x:+6.2f}" for x in v) + "]"


def main(argv: list[str] | None = None) -> int:
    """Serve the page until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8098)
    parser.add_argument("--rate", type=float, default=20.0, help="redraw rate, Hz")
    parser.add_argument("--stale-s", type=float, default=0.5)
    parser.add_argument("--tau-s", type=float, default=0.1, help="initial low-pass time constant")
    # Colour scales only. 13 N is a 10 mm press at the 1300 N/m translational stiffness.
    parser.add_argument("--full-scale-n", type=float, default=15.0)
    parser.add_argument("--full-scale-nm", type=float, default=10.0)
    args = parser.parse_args(argv)

    rclpy.init()
    try:
        node = ContactView(args)
    except SystemExit as refusal:
        print(f"refusing: {refusal}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
