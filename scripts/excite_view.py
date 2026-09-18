"""A live viser view of the excitation run: what was commanded, and what the arm did.

Two arms are drawn. The **ghost** is the plan -- where the trajectory says the arm should be at
this instant. The **solid** one is the arm, from the joint states the bridge replies with. The
gap between them is the thing this whole exercise turns on.

It makes the open question visible rather than statistical. The Cartesian controller has 3 Nm of
nullspace authority, and in 58% of the planned configurations gravity along the elbow direction
exceeds it. If that is not enough, the ghost's elbow swings and the real elbow does not -- while
the tip of both tracks fine, because the task term has plenty of authority and only the redundant
degree of freedom is starved. That failure produces no error anywhere: the pose tracks, the plan
completes, the recording looks healthy, and the seventh dimension is simply missing from the data.

Here you would see it in a second.
"""

from __future__ import annotations

from functools import partial
from pathlib import Path

import numpy as np

__all__ = ["LiveView"]

#: The plan, drawn translucent. Alpha low enough to read the solid arm through it.
GHOST_RGBA = (0.35, 0.55, 0.95, 0.30)

#: The arm itself, drawn solid in a warm tone so the two never read as the same object.
REAL_RGBA = (0.85, 0.55, 0.25, 1.0)


def _load_urdf(urdf: Path, mesh_dirs: list[Path] | None):  # noqa: ANN202 - yourdfpy.URDF
    """Load the description with ``package://`` resolved against the mesh roots.

    pinocchio's loader wants the directory *containing* the package and yourdfpy's own helper
    wants the package directory itself, so both are tried. Handed neither, ``ViserUrdf`` renders
    an arm with no geometry and prints one line per missing mesh -- which reads as a broken
    viewer rather than an unset path.
    """
    import yourdfpy

    roots = [Path(d) for d in (mesh_dirs or [])]
    if not roots:
        return yourdfpy.URDF.load(str(urdf))

    def resolve(fname: str) -> str:
        if not fname.startswith("package://"):
            return fname
        stripped = fname[len("package://") :]
        _, _, rest = stripped.partition("/")
        for root in roots:
            for candidate in (root / stripped, root / rest):
                if candidate.exists():
                    return str(candidate)
        return fname

    return yourdfpy.URDF.load(str(urdf), filename_handler=resolve)


class LiveView:
    """Serves the two-arm view and the readouts beside it."""

    def __init__(
        self,
        urdf: Path,
        mesh_dirs: list[Path] | None,
        plan_tips: np.ndarray,
        box: tuple[np.ndarray, np.ndarray] | None = None,
        host: str = "0.0.0.0",
        port: int = 8095,
    ) -> None:
        """Start the server and build the scene.

        Args:
            urdf: Robot description.
            mesh_dirs: Roots for ``package://`` references.
            plan_tips: ``(n, 3)`` tip positions of the whole plan, drawn once as a path.
            box: Workspace box corners, drawn as a wireframe so the safety limit is visible.
            host: Bind address. Every interface, so the bench can be watched from a laptop.
            port: HTTP port.
        """
        import viser
        from viser.extras import ViserUrdf

        self.server = viser.ViserServer(host=host, port=port)
        self.server.scene.add_grid("/floor", width=3.0, height=3.0, cell_size=0.25)
        if box is not None:
            lo, hi = box
            self.server.scene.add_box(
                "/box",
                dimensions=tuple(hi - lo),
                position=tuple((hi + lo) / 2),
                color=(90, 90, 110),
                wireframe=True,
            )

        loaded = partial(_load_urdf, urdf, mesh_dirs)
        # Two independent loads: ViserUrdf mutates the model it is given when it updates the
        # configuration, so sharing one between both arms would make them move together and the
        # view would show perfect tracking no matter what the arm did.
        self.ghost = ViserUrdf(
            self.server,
            urdf_or_path=loaded(),
            root_node_name="/plan",
            mesh_color_override=GHOST_RGBA,
        )
        self.real = ViserUrdf(
            self.server,
            urdf_or_path=loaded(),
            root_node_name="/arm",
            mesh_color_override=REAL_RGBA,
        )

        # The whole session's tip path, drawn once. A dense knot in one corner reads instantly
        # here and survives no summary statistic.
        stride = max(1, len(plan_tips) // 4000)
        self.server.scene.add_point_cloud(
            "/path",
            points=np.asarray(plan_tips[::stride], dtype=np.float32),
            colors=np.tile(np.array([120, 150, 200], dtype=np.uint8), (len(plan_tips[::stride]), 1)),
            point_size=0.003,
        )

        # Where the plan is asking the tip to be, right now.
        self.target_frame = self.server.scene.add_frame(
            "/target", axes_length=0.12, axes_radius=0.005
        )

        with self.server.gui.add_folder("run"):
            self.gui_phase = self.server.gui.add_text("phase", initial_value="", disabled=True)
            self.gui_index = self.server.gui.add_text("sample", initial_value="", disabled=True)
            self.gui_joint = self.server.gui.add_text(
                "worst joint error", initial_value="", disabled=True
            )
            self.gui_elbow = self.server.gui.add_text(
                "elbow (A3) plan vs arm", initial_value="", disabled=True
            )
            self.gui_tip = self.server.gui.add_text("tip error", initial_value="", disabled=True)

        print(f"live view on http://{host}:{port}")

    def update(
        self,
        q_plan: np.ndarray,
        q_measured: np.ndarray | None,
        target_position: np.ndarray,
        target_quaternion: np.ndarray,
        measured_position: np.ndarray | None,
        index: int,
        total: int,
        phase: str,
    ) -> None:
        """Redraw. Cheap enough to call at the streaming rate; callers may throttle."""
        self.ghost.update_cfg(np.asarray(q_plan, dtype=float))
        x, y, z, w = target_quaternion
        self.target_frame.position = tuple(float(v) for v in target_position)
        self.target_frame.wxyz = (float(w), float(x), float(y), float(z))

        self.gui_phase.value = phase
        self.gui_index.value = f"{index} / {total}"

        if q_measured is None:
            self.gui_joint.value = "no joint state"
            return
        self.real.update_cfg(np.asarray(q_measured, dtype=float))

        error = np.degrees(np.abs(np.asarray(q_plan) - np.asarray(q_measured)))
        worst = int(np.argmax(error))
        self.gui_joint.value = f"A{worst + 1}  {error[worst]:.2f} deg"
        # A3 called out by name because it is the elbow-ish joint the nullspace term has to move,
        # and it is the one that quietly fails to follow when 3 Nm is not enough.
        self.gui_elbow.value = (
            f"{np.degrees(q_plan[2]):+7.2f} vs {np.degrees(q_measured[2]):+7.2f} deg"
            f"   (off by {error[2]:.2f})"
        )
        if measured_position is not None:
            gap = float(np.linalg.norm(np.asarray(target_position) - measured_position)) * 1e3
            self.gui_tip.value = f"{gap:.1f} mm"
