"""Drag the wrist camera in a browser, and the arm follows it.

For a stack launched with `tool:=umi` (`pixi run -e jazzy hardware-umi`), where the Cartesian
endpoint IS `lbr_umi_camera`. A viser page draws the arm from its own robot_description, the
measured camera frame, the target being sent, and a transform gizmo sitting on the camera.
Engage, drag the gizmo's axes, and the camera goes there -- the same frame a policy's
SE(3)-relative chunks compose onto, which is the point: before a policy drives this arm, a
human should have driven it in exactly the terms the policy will use, and seen that "+z" means
forward along the optical axis.

It refuses to run against the wrong frame, and it checks that twice. At start it reads
`end_effector_frame` back off `pose_broadcaster` and `cartesian_impedance_controller`; every
tick it compares /lbr/current_pose with its own forward kinematics of `lbr_umi_camera` from the
live robot_description. The second one is the check that matters: on a `tool:=none` stack the
two differ by ~90 mm and 25 deg, and every other number here would look healthy.

What reaches /lbr/target_pose is bounded the way haply_teleop_bridge.py bounds it, with tighter
defaults because a mouse can jump a gizmo a metre in one event:

* engaging sets the target to the MEASURED pose and snaps the gizmo onto it, so engage is a
  no-op and the first published target is where the arm already is;
* the target walks toward the gizmo at --max-speed-m-s / --max-rot-deg-s, not in one step;
* it is held within --max-error-m / --max-error-deg of the measured pose. That is the force
  bound: 1300 N/m x 0.03 m = 39 N;
* its position is clamped into a workspace box in lbr_link_0, drawn in the scene;
* a stale pose (> --stale-ms), a lost controller, a frame mismatch or a second publisher on
  the topic disengages. Disengaged means NOT PUBLISHING: the controller holds its last target
  under impedance.

Two corrections, off until ticked under "correction", for the ~1 cm the arm trails by: an
integral that absorbs the static offset (stiction, load-data error), and a velocity lead of
(D/K) v that cancels the damping drag while moving. Both are added AFTER the walk and BEFORE the
bounds above, so neither can push past the force bound or the box. "gizmo - arm, 2 s" is the
number to compare with them on and off.

Arming and resting are buttons, and they are the same two STRICT switches as
kuka_crisp.safe_switch -- REST (trajectory + zero effort) <-> ARMED (passthrough + Cartesian) --
issued here directly on the controller_manager service. Not through crisp_py: its Robot starts a
50 Hz target_pose publisher on construction, and this node is already a publisher on that topic,
so the controller would see two and ignore both (cartesian_controller.cpp:894).
CartesianController applies a target that arrived while it was INACTIVE on its first active
tick (new_target_pose_ is not reset in on_activate), so this node publishes nothing unless the
overlay is active.

    pixi run -e jazzy hardware-umi                                   # the stack, camera endpoint
    pixi run -e jazzy python scripts/camera_gizmo.py --dry-run       # publishes nothing
    pixi run -e jazzy python scripts/camera_gizmo.py                 # open the printed link
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import String

NS = "/lbr"
TARGET_TOPIC = f"{NS}/target_pose"
CAMERA_FRAME = "lbr_umi_camera"
BASE_FRAME = "lbr_link_0"
JOINT_NAMES = tuple(f"lbr_A{i + 1}" for i in range(7))

CARTESIAN = "cartesian_impedance_controller"
PASSTHROUGH = "fri_position_passthrough_controller"
TRAJECTORY = "joint_trajectory_controller"
ZERO_EFFORT = "zero_effort_controller"
FRAME_CHECKED = ("pose_broadcaster", CARTESIAN)

#: D405 colour stream, for the frustum only: it shows which way the optical axis points.
FRUSTUM_FOV_RAD = np.radians(58.0)
FRUSTUM_ASPECT = 16 / 9

Pose = tuple[np.ndarray, Rotation]


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


class CameraGizmo(Node):
    """The loop that turns a dragged gizmo into bounded targets for the camera frame."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Wire up topics and services; verify the frame; publish nothing yet."""
        super().__init__("camera_gizmo")
        self.args = args
        self.lock = threading.Lock()
        self.measured: Pose | None = None
        self.measured_at: float | None = None
        self.q: np.ndarray | None = None
        self.q_at: float | None = None
        self.commanded: Pose | None = None
        # Where the arm SHOULD be: the gizmo, walked at the slider speeds. `commanded` is this
        # plus the corrections, and is what is published.
        self.reference: Pose | None = None
        self.i_pos = np.zeros(3)
        self.i_rot = np.zeros(3)
        self.saturated = False
        self.error_log: list[tuple[float, float, float]] = []  # (t, mm, deg) reference vs arm
        self.gains: dict[str, float] = {}
        self.engaged = False
        self.engage_requested = False
        self.status = "disengaged"
        self.active: set[str] = set()
        self.active_at: float | None = None
        self.other_publishers = 0
        self.frame_error_mm: float | None = None
        self.clamped = {"speed": 0, "rot": 0, "error": 0, "box": 0}
        self.published = 0
        self.busy = False  # a controller switch is in flight

        self.box_lo = np.array(args.workspace_min, dtype=float)
        self.box_hi = np.array(args.workspace_max, dtype=float)

        self.pub = self.create_publisher(PoseStamped, TARGET_TOPIC, 1)
        self.create_subscription(
            PoseStamped, f"{NS}/current_pose", self._on_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, f"{NS}/joint_states", self._on_joints, qos_profile_sensor_data
        )
        self.list_client = self.create_client(
            ListControllers, f"{NS}/controller_manager/list_controllers"
        )
        self.switch_client = self.create_client(
            SwitchController, f"{NS}/controller_manager/switch_controller"
        )

        self.urdf_xml = self._wait_for_description()
        self.model = pin.buildModelFromXML(self.urdf_xml)
        self.data = self.model.createData()
        if not self.model.existFrame(CAMERA_FRAME):
            raise SystemExit(
                f"robot_description has no {CAMERA_FRAME}: this stack was launched without the "
                f"UMI. Relaunch with `pixi run -e jazzy hardware-umi` (tool:=umi)."
            )
        self.camera_id = self.model.getFrameId(CAMERA_FRAME)
        self._check_controller_frames()

        self._build_scene()
        self.create_timer(1.0 / args.rate, self._tick)
        self.create_timer(1.0, self._poll)
        self.create_timer(0.1, self._refresh_view)

    # -- startup checks -----------------------------------------------------------------

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

    def _check_controller_frames(self) -> None:
        """Read end_effector_frame back off the controllers that define the endpoint."""
        for controller in FRAME_CHECKED:
            client = self.create_client(GetParameters, f"{NS}/{controller}/get_parameters")
            if not client.wait_for_service(timeout_sec=5.0):
                raise SystemExit(f"{controller} is not loaded; cannot read its end_effector_frame")
            names = ["end_effector_frame"]
            if controller == CARTESIAN:
                names += ["task.k_pos_x", "task.d_pos_x", "task.k_rot_x", "task.d_rot_x"]
            request = GetParameters.Request(names=names)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is None or not future.result().values:
                raise SystemExit(f"could not read {controller}.end_effector_frame")
            frame = future.result().values[0].string_value
            if frame != CAMERA_FRAME:
                raise SystemExit(
                    f"{controller}.end_effector_frame is {frame!r}, not {CAMERA_FRAME!r}. "
                    f"Targets from here would move {frame} to where the camera was meant to go. "
                    f"Relaunch with tool:=umi."
                )
            for name, value in zip(names[1:], future.result().values[1:]):
                self.gains[name] = value.double_value
            self.destroy_client(client)
            print(f"  {controller}.end_effector_frame = {frame}", flush=True)
        # The lead that cancels the damping drag. crisp damps against MEASURED velocity with the
        # target's velocity taken as zero, so a target moving at v holds a steady-state error of
        # (D/K) v behind it; sending the target (D/K) v ahead of itself cancels that.
        self.lead_pos_s = self.gains["task.d_pos_x"] / self.gains["task.k_pos_x"]
        self.lead_rot_s = self.gains["task.d_rot_x"] / self.gains["task.k_rot_x"]
        print(
            f"  gains k_pos {self.gains['task.k_pos_x']:.0f} d_pos {self.gains['task.d_pos_x']:.1f}"
            f" -> lead {self.lead_pos_s * 1e3:.0f} ms; k_rot {self.gains['task.k_rot_x']:.0f}"
            f" d_rot {self.gains['task.d_rot_x']:.1f} -> lead {self.lead_rot_s * 1e3:.0f} ms",
            flush=True,
        )

    # -- inputs -------------------------------------------------------------------------

    def _on_pose(self, msg: PoseStamped) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        with self.lock:
            self.measured = (np.array([p.x, p.y, p.z]), Rotation.from_quat([o.x, o.y, o.z, o.w]))
            self.measured_at = time.monotonic()

    def _on_joints(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        if all(n in by_name for n in JOINT_NAMES):
            with self.lock:
                self.q = np.array([by_name[n] for n in JOINT_NAMES])
                self.q_at = time.monotonic()

    def _poll(self) -> None:
        """Once a second: which controllers are active, and who else publishes targets."""
        self.other_publishers = max(0, self.count_publishers(TARGET_TOPIC) - 1)
        if not self.list_client.service_is_ready():
            return
        future = self.list_client.call_async(ListControllers.Request())

        def done(f) -> None:
            if f.result() is not None:
                self.active = {c.name for c in f.result().controller if c.state == "active"}
                self.active_at = time.monotonic()

        future.add_done_callback(done)

    # -- kinematics ---------------------------------------------------------------------

    def _fk_camera(self, q: np.ndarray) -> Pose:
        full = pin.neutral(self.model)
        for name, value in zip(JOINT_NAMES, q):
            full[self.model.joints[self.model.getJointId(name)].idx_q] = value
        pin.framesForwardKinematics(self.model, self.data, full)
        placement = self.data.oMf[self.camera_id]
        return placement.translation.copy(), Rotation.from_matrix(placement.rotation.copy())

    # -- the loop -----------------------------------------------------------------------

    def _fault(self, now: float) -> str | None:
        """Why the arm must not be commanded right now, or None."""
        if self.measured_at is None or now - self.measured_at > self.args.stale_ms / 1e3:
            return "current_pose is stale"
        if self.q_at is None or now - self.q_at > self.args.stale_ms / 1e3:
            return "joint_states are stale"
        if self.active_at is None or now - self.active_at > 3.0:
            return "controller list unknown"
        if not {CARTESIAN, PASSTHROUGH} <= self.active:
            return f"{CARTESIAN} is not active -- press 'arm' first"
        if self.other_publishers:
            return f"{self.other_publishers} other publisher(s) on {TARGET_TOPIC}"
        if self.frame_error_mm is None or self.frame_error_mm > self.args.frame_tol_mm:
            return f"current_pose is not {CAMERA_FRAME} (off by {self.frame_error_mm} mm)"
        return None

    def _tick(self) -> None:
        now = time.monotonic()
        with self.lock:
            measured, q = self.measured, self.q
        if measured is not None and q is not None:
            fk_p, _ = self._fk_camera(q)
            self.frame_error_mm = float(np.linalg.norm(fk_p - measured[0]) * 1e3)

        if self.engage_requested:
            self.engage_requested = False
            reason = self._fault(now)
            if reason:
                self.status = f"refused: {reason}"
                self.gui_engage.value = False
            else:
                self.commanded = (measured[0].copy(), measured[1])
                self.reference = self.commanded
                self._reset_corrections()
                self._snap_gizmo(measured)
                self.engaged = True
                self.status = "ENGAGED -- drag the gizmo"

        if not self.engaged:
            return
        reason = self._fault(now)
        if reason:
            self._disengage(f"disengaged: {reason}")
            return

        goal = (np.array(self.gizmo.position, dtype=float), _rot_wxyz(self.gizmo.wxyz))
        self.commanded = self._limit(goal, measured)
        if not self.args.dry_run:
            self._publish(self.commanded)

    def _limit(self, goal: Pose, measured: Pose) -> Pose:
        """Walk the reference toward the gizmo, add the corrections, bound what is sent."""
        previous = self.reference
        self.reference = self._bound(self._walk(goal, previous), measured, count=False)
        self._log_error(self.reference, measured)
        return self._bound(self._correct(self.reference, previous, measured), measured)

    def _walk(self, goal: Pose, previous: Pose) -> Pose:
        """Step from the previous reference toward the gizmo at the slider speeds."""
        prev_p, prev_r = previous
        position, rotation = goal

        step = position - prev_p
        dist = float(np.linalg.norm(step))
        max_step = self.gui_speed.value / self.args.rate
        if dist > max_step:
            position = prev_p + step * (max_step / dist)
            self.clamped["speed"] += 1

        delta = rotation * prev_r.inv()
        ang = float(delta.magnitude())
        max_ang = np.radians(self.gui_rot.value) / self.args.rate
        if ang > max_ang:
            rotation = Rotation.from_rotvec(delta.as_rotvec() * (max_ang / ang)) * prev_r
            self.clamped["rot"] += 1
        return position, rotation

    def _correct(self, reference: Pose, previous: Pose, measured: Pose) -> Pose:
        """The two client-side corrections, both off unless ticked.

        integral: the arm stops SHORT of a static target -- stiction holds the proximal joints
        (A1 breaks away near 8 Nm), and whatever the UMI's declared load data gets wrong is left
        on the spring. (Undeclared, its ~0.4 kg sagged ~3 mm at 1300 N/m; the ~1 cm trail this
        was built against was measured then.) crisp has no integral term, so this integrates the
        reference-to-arm error into an offset on what is sent. A deadband keeps it from hunting
        on stiction, a cap bounds it, and it freezes while the output is being force-clamped.

        lead: the arm LAGS a moving target by (D/K) v, because crisp's damping acts on measured
        velocity against a zero target velocity. Sending the reference ahead by (D/K) v cancels
        it at constant speed.
        """
        ref_p, ref_r = reference
        out_p, out_rv = ref_p.copy(), np.zeros(3)

        if self.gui_integral.value and not self.saturated:
            dt = 1.0 / self.args.rate
            ki = self.gui_ki.value
            e_p = ref_p - measured[0]
            if np.linalg.norm(e_p) > self.args.deadband_mm / 1e3:
                self.i_pos += ki * e_p * dt
            e_r = (ref_r * measured[1].inv()).as_rotvec()
            if np.linalg.norm(e_r) > np.radians(self.args.deadband_deg):
                self.i_rot += ki * e_r * dt
            self.i_pos = _cap(self.i_pos, self.args.max_integral_mm / 1e3)
            self.i_rot = _cap(self.i_rot, np.radians(self.args.max_integral_deg))
        if self.gui_integral.value:
            out_p += self.i_pos
            out_rv += self.i_rot

        if self.gui_lead.value:
            v = (ref_p - previous[0]) * self.args.rate
            w = (ref_r * previous[1].inv()).as_rotvec() * self.args.rate
            out_p += self.lead_pos_s * v
            out_rv += self.lead_rot_s * w

        return out_p, Rotation.from_rotvec(out_rv) * ref_r

    def _bound(self, pose: Pose, measured: Pose, *, count: bool = True) -> Pose:
        """The workspace box, and the force bound: F = k (target - measured), so cap the gap."""
        position, rotation = pose
        saturated = False
        clipped = np.clip(position, self.box_lo, self.box_hi)
        if not np.allclose(clipped, position):
            self.clamped["box"] += count
        position = clipped

        gap = position - measured[0]
        dist = float(np.linalg.norm(gap))
        if dist > self.args.max_error_m:
            position = measured[0] + gap * (self.args.max_error_m / dist)
            saturated = True
        delta_r = rotation * measured[1].inv()
        ang_r = float(delta_r.magnitude())
        max_ang_r = np.radians(self.args.max_error_deg)
        if ang_r > max_ang_r:
            rotation = Rotation.from_rotvec(delta_r.as_rotvec() * (max_ang_r / ang_r)) * measured[1]
            saturated = True
        if count:
            self.clamped["error"] += saturated
            self.saturated = saturated
        return position, rotation

    def _reset_corrections(self) -> None:
        self.i_pos = np.zeros(3)
        self.i_rot = np.zeros(3)
        self.saturated = False
        self.error_log.clear()

    def _log_error(self, reference: Pose, measured: Pose) -> None:
        now = time.monotonic()
        mm = float(np.linalg.norm(reference[0] - measured[0]) * 1e3)
        deg = float(np.degrees((reference[1] * measured[1].inv()).magnitude()))
        self.error_log.append((now, mm, deg))
        while self.error_log and now - self.error_log[0][0] > 2.0:
            self.error_log.pop(0)

    def _publish(self, pose: Pose) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in pose[0])
        x, y, z, w = pose[1].as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
        self.pub.publish(msg)
        self.published += 1

    def _disengage(self, status: str) -> None:
        self.engaged = False
        self.commanded = None
        self.reference = None
        self._reset_corrections()
        self.status = status
        self.gui_engage.value = False
        print(f"  {status} -- publication stopped, the arm holds its last target")

    # -- controller switches ------------------------------------------------------------

    def _switch(self, arm: bool) -> None:
        """REST <-> ARMED as one STRICT switch, off the ROS thread so the loop keeps ticking."""
        if self.busy:
            return
        if self.engaged:
            self._disengage("disengaged for a controller switch")
        active = set(self.active)
        if arm:
            if {CARTESIAN, PASSTHROUGH} <= active:
                self.status = "already armed"
                return
            if not {TRAJECTORY, ZERO_EFFORT} <= active:
                self.status = f"refused: not at REST ({sorted(active)})"
                return
            if self.other_publishers:
                self.status = f"refused: {self.other_publishers} other publisher(s) on target_pose"
                return
            activate, deactivate = [PASSTHROUGH, CARTESIAN], [TRAJECTORY, ZERO_EFFORT]
        else:
            if {TRAJECTORY, ZERO_EFFORT} <= active:
                self.status = "already at rest"
                return
            if PASSTHROUGH not in active:
                self.status = f"refused: nothing owns the position command ({sorted(active)})"
                return
            overlays = sorted(active & {CARTESIAN, "joint_impedance_controller"})
            activate, deactivate = [TRAJECTORY, ZERO_EFFORT], [PASSTHROUGH, *overlays]

        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = rclpy.duration.Duration(seconds=10).to_msg()
        self.busy = True
        self.status = f"switching: +{activate} -{deactivate}"
        future = self.switch_client.call_async(request)

        def done(f) -> None:
            self.busy = False
            ok = f.result() is not None and f.result().ok
            self.status = ("ARMED" if arm else "REST") if ok else "STRICT switch FAILED"
            print(f"  switch +{activate} -{deactivate}: {'ok' if ok else 'FAILED'}")
            self.active_at = None  # force a fresh read before anything engages

        future.add_done_callback(done)

    # -- the page -----------------------------------------------------------------------

    def _build_scene(self) -> None:
        import viser
        import yourdfpy
        from viser.extras import ViserUrdf

        self.server = viser.ViserServer(host=self.args.host, port=self.args.port)
        scene, gui = self.server.scene, self.server.gui
        scene.add_grid("/floor", width=2.0, height=2.0, cell_size=0.1)

        with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
            f.write(self.urdf_xml)
        urdf = yourdfpy.URDF.load(f.name, filename_handler=_resolve_package_uri)
        self.viser_urdf = ViserUrdf(self.server, urdf_or_path=urdf, root_node_name="/arm")
        self.actuated = list(self.viser_urdf.get_actuated_joint_names())

        scene.add_box(
            "/workspace",
            dimensions=tuple(self.box_hi - self.box_lo),
            position=tuple((self.box_hi + self.box_lo) / 2),
            color=(90, 90, 110),
            wireframe=True,
        )
        # Measured camera, with a frustum along its +z so the optical axis is visible.
        self.measured_frame = scene.add_frame("/measured", axes_length=0.06, axes_radius=0.003)
        scene.add_camera_frustum(
            "/measured/frustum",
            fov=FRUSTUM_FOV_RAD,
            aspect=FRUSTUM_ASPECT,
            scale=0.06,
            color=(240, 180, 60),
        )
        self.target_frame = scene.add_frame(
            "/target", axes_length=0.04, axes_radius=0.002, visible=False
        )
        self.gizmo = scene.add_transform_controls("/gizmo", scale=0.15, line_width=3.0)

        with gui.add_folder("arm"):
            self.gui_status = gui.add_text("status", initial_value=self.status, disabled=True)
            self.gui_ctrl = gui.add_text("controllers", initial_value="", disabled=True)
            self.gui_frame = gui.add_text("frame check", initial_value="", disabled=True)
            arm_button = gui.add_button("arm (REST -> Cartesian)")
            rest_button = gui.add_button("rest (hold on trajectory ctrl)")
        with gui.add_folder("drive"):
            self.gui_engage = gui.add_checkbox("engaged", initial_value=False)
            snap = gui.add_button("snap gizmo to camera")
            self.gui_speed = gui.add_slider(
                "max speed m/s", min=0.005, max=self.args.max_speed_m_s, step=0.005,
                initial_value=min(0.05, self.args.max_speed_m_s),
            )
            self.gui_rot = gui.add_slider(
                "max rot deg/s", min=1.0, max=self.args.max_rot_deg_s, step=1.0,
                initial_value=min(15.0, self.args.max_rot_deg_s),
            )
            self.gui_gap = gui.add_text("target - arm", initial_value="", disabled=True)
            self.gui_lag = gui.add_text("gizmo - target", initial_value="", disabled=True)
            self.gui_clamps = gui.add_text("clamps", initial_value="", disabled=True)
        with gui.add_folder("correction"):
            self.gui_track = gui.add_text("gizmo - arm, 2 s", initial_value="", disabled=True)
            self.gui_integral = gui.add_checkbox("integral (static offset)", initial_value=False)
            self.gui_ki = gui.add_slider("ki 1/s", min=0.1, max=5.0, step=0.1, initial_value=1.0)
            self.gui_lead = gui.add_checkbox("velocity lead (D/K v)", initial_value=False)
            self.gui_offset = gui.add_text("integral now", initial_value="", disabled=True)
        with gui.add_folder("nudge (camera axes)"):
            self.gui_step = gui.add_slider(
                "step mm", min=1.0, max=50.0, step=1.0, initial_value=10.0
            )
            for axis, label in enumerate("xyz"):
                row = gui.add_button_group(f"cam {label}", options=[f"-{label}", f"+{label}"])
                row.on_click(lambda e, a=axis: self._nudge(a, 1 if e.target.value[0] == "+" else -1))

        @self.gui_engage.on_update
        def _(_) -> None:
            if self.gui_engage.value and not self.engaged:
                self.engage_requested = True
            elif not self.gui_engage.value and self.engaged:
                self._disengage("disengaged by operator")

        arm_button.on_click(lambda _: self._switch(arm=True))
        rest_button.on_click(lambda _: self._switch(arm=False))
        snap.on_click(lambda _: self._snap_gizmo(self.measured))

        @self.server.on_client_connect
        def _(client) -> None:
            # Open looking at the camera from behind and above it, not at the whole cell: the
            # gizmo is 15 cm across and the default view shows the arm from two metres.
            if self.measured is not None:
                p = self.measured[0]
                client.camera.position = tuple(p + np.array([-0.6, -0.5, 0.4]))
                client.camera.look_at = tuple(p)

        for link in _lan_links(self.args.port):
            print(f"  camera gizmo on {link}", flush=True)

    def _nudge(self, axis: int, sign: int) -> None:
        """Move the gizmo along one of its own axes -- which are the camera's."""
        rotation = _rot_wxyz(self.gizmo.wxyz)
        offset = rotation.as_matrix()[:, axis] * sign * self.gui_step.value / 1e3
        self.gizmo.position = tuple(np.asarray(self.gizmo.position) + offset)

    def _snap_gizmo(self, pose: Pose | None) -> None:
        if pose is None:
            return
        self.gizmo.position = tuple(float(v) for v in pose[0])
        self.gizmo.wxyz = _wxyz(pose[1])

    def _refresh_view(self) -> None:
        with self.lock:
            measured, q = self.measured, self.q
        if q is not None:
            cfg = np.zeros(len(self.actuated))
            for i, name in enumerate(self.actuated):
                if name in JOINT_NAMES:
                    cfg[i] = q[JOINT_NAMES.index(name)]
            self.viser_urdf.update_cfg(cfg)
        if measured is not None:
            self.measured_frame.position = tuple(float(v) for v in measured[0])
            self.measured_frame.wxyz = _wxyz(measured[1])
            if not self.engaged and not self.busy:
                # Disengaged, the gizmo rides on the camera: engaging starts from zero offset.
                self._snap_gizmo(measured)

        self.gui_status.value = self.status + (" [DRY RUN]" if self.args.dry_run else "")
        armed = {CARTESIAN, PASSTHROUGH} <= self.active
        rest = {TRAJECTORY, ZERO_EFFORT} <= self.active
        self.gui_ctrl.value = "ARMED" if armed else "REST" if rest else sorted(self.active).__str__()
        if self.frame_error_mm is not None:
            ok = self.frame_error_mm <= self.args.frame_tol_mm
            self.gui_frame.value = f"{'ok' if ok else 'MISMATCH'} {self.frame_error_mm:.2f} mm"

        if self.commanded is not None and measured is not None:
            self.target_frame.visible = True
            self.target_frame.position = tuple(float(v) for v in self.commanded[0])
            self.target_frame.wxyz = _wxyz(self.commanded[1])
            gap = np.linalg.norm(self.commanded[0] - measured[0]) * 1e3
            ang = np.degrees((self.commanded[1] * measured[1].inv()).magnitude())
            self.gui_gap.value = f"{gap:.1f} mm  {ang:.1f} deg"
            lag = np.linalg.norm(np.asarray(self.gizmo.position) - self.commanded[0]) * 1e3
            self.gui_lag.value = f"{lag:.1f} mm"
        else:
            self.target_frame.visible = False
            self.gui_gap.value = self.gui_lag.value = ""
        if self.error_log:
            mm = np.array([e[1] for e in self.error_log])
            deg = np.array([e[2] for e in self.error_log])
            self.gui_track.value = (
                f"mean {mm.mean():.1f} max {mm.max():.1f} mm | mean {deg.mean():.2f} deg"
            )
        else:
            self.gui_track.value = ""
        self.gui_offset.value = (
            f"{np.linalg.norm(self.i_pos) * 1e3:.1f} mm  {np.degrees(np.linalg.norm(self.i_rot)):.2f} deg"
        )
        c = self.clamped
        self.gui_clamps.value = (
            f"speed {c['speed']} rot {c['rot']} err {c['error']} box {c['box']} | pub {self.published}"
        )


def _cap(vector: np.ndarray, limit: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector * (limit / norm) if norm > limit else vector


def _wxyz(rotation: Rotation) -> tuple[float, float, float, float]:
    x, y, z, w = rotation.as_quat()
    return float(w), float(x), float(y), float(z)


def _rot_wxyz(wxyz: tuple[float, float, float, float]) -> Rotation:
    w, x, y, z = wxyz
    return Rotation.from_quat([x, y, z, w])


def main(argv: list[str] | None = None) -> int:
    """Serve the page and run the loop until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8097)
    parser.add_argument("--rate", type=float, default=100.0, help="publish rate, Hz")
    parser.add_argument("--dry-run", action="store_true", help="everything but publishing")
    parser.add_argument("--stale-ms", type=float, default=250.0)
    # current_pose against FK(lbr_umi_camera) on the same description: they agree to 0.1 mm in
    # mock (21 Sep 2026), and a tool:=none stack is ~90 mm off. Anything past a few mm is the
    # wrong frame, not noise.
    parser.add_argument("--frame-tol-mm", type=float, default=2.0)
    # Slider ceilings. The sliders start at 0.05 m/s and 15 deg/s.
    parser.add_argument("--max-speed-m-s", type=float, default=0.25)
    parser.add_argument("--max-rot-deg-s", type=float, default=60.0)
    # The force bound: 1300 N/m (config/controllers.yaml) x 0.03 m = 39 N.
    parser.add_argument("--max-error-m", type=float, default=0.03)
    parser.add_argument("--max-error-deg", type=float, default=10.0)
    # The integral's cap: what it is there to absorb is stiction plus load-data error, a few mm, so
    # 15 mm is room for that with margin, and small against the 30 mm force bound it sits under.
    parser.add_argument("--max-integral-mm", type=float, default=15.0)
    parser.add_argument("--max-integral-deg", type=float, default=5.0)
    # Inside this the integral does not accumulate: stiction plus an integrator is a limit cycle.
    parser.add_argument("--deadband-mm", type=float, default=0.5)
    parser.add_argument("--deadband-deg", type=float, default=0.2)
    # A box for the CAMERA in lbr_link_0, around the policy-eval home captured 21 Sep 2026
    # (lerobot_pickplace configs/home.kuka.json, camera at 0.518, -0.432, 0.512). The floor is
    # measured: the table is at z 0.276 under the camera (D405 depth, RANSAC plane, 1.9 mm rms)
    # and the tweezer tips hang 137 mm below the camera there, so z 0.425 keeps them >= 7 mm
    # off the table. Same box as lerobot_pickplace configs/deploy.kuka.yaml; keep them equal.
    parser.add_argument("--workspace-min", type=float, nargs=3, default=[0.30, -0.70, 0.425])
    parser.add_argument("--workspace-max", type=float, nargs=3, default=[0.75, -0.20, 0.70])
    args = parser.parse_args(argv)

    rclpy.init()
    try:
        node = CameraGizmo(args)
    except SystemExit as refusal:
        print(f"refusing: {refusal}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(
            "stopping -- nothing more is published; if ARMED the arm holds its last target. "
            "To rest: pixi run -e jazzy session --switch zero_effort_controller",
            file=sys.stderr,
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
