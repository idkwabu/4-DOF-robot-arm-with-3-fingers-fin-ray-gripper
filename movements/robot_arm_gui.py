"""
DUM-E Robot Arm — Tkinter GUI (5 core workflow stages).

    Forward Kinematics | Inverse Kinematics | Reachability
    | Verification     | Visualization

Run it with:

    python robot_arm_gui.py

The Visualization tab embeds the SAME matplotlib 3D renderer as the CLI
(``robot_arm.draw_robot``), so the picture inside the UI is identical to
``plot_robot()``.  Every stage accepts an optional gripper opening
(0 = closed, fingers meet on the tool axis; 1 = fully open); ``Enter``/blank
keeps the config default ``gripper_opening``.

Requires a Tk-capable Python (the standard Windows/macOS installers include
tkinter; on Debian/Ubuntu: ``sudo apt install python3-tk``) plus numpy and
matplotlib.
"""

from __future__ import annotations

import contextlib
import io
import math
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Optional

import robot_arm as ra

import matplotlib

try:
    matplotlib.use("TkAgg")  # must be set before importing pyplot
    import matplotlib.pyplot as plt  # noqa: E402,F401
    from matplotlib.backends.backend_tkagg import (  # noqa: E402
        FigureCanvasTkAgg,
        NavigationToolbar2Tk,
    )
    from matplotlib.figure import Figure  # noqa: E402
    from mpl_toolkits.mplot3d import Axes3D  # noqa: E402,F401  (registers 3d)
except Exception as exc:  # pragma: no cover - environment dependent
    raise SystemExit(
        "The tkinter/matplotlib-Tk stack could not be initialised.\n"
        f"Reason: {exc}\n"
        "Hint: on Linux install python3-tk; otherwise use the CLI "
        "(python robot_arm.py)."
    ) from exc

DEFAULT_CFG = ra.RobotConfig()

VIZ_ANIM_FRAMES = 30
VIZ_ANIM_INTERVAL_MS = 30


def _same_pose(a: ra.JointAngles, b: ra.JointAngles, tol: float = 1e-3) -> bool:
    """True when two poses are (numerically) the same joint position."""
    return all(
        abs(getattr(a, f) - getattr(b, f)) <= tol
        for f in ("theta_base", "joint1", "joint2", "joint3")
    )


class _Output(ttk.Frame):
    """A scrollable, read-only text area for a tab's results."""

    def __init__(self, parent, height: int = 11) -> None:
        super().__init__(parent)
        self.text = tk.Text(
            self, height=height, state="disabled", wrap="word", font=("Consolas", 10)
        )
        scroll = ttk.Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=scroll.set)
        self.text.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

    def show(self, message: str) -> None:
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", message.rstrip() + "\n")
        self.text.configure(state="disabled")


class _Field(ttk.Frame):
    """A labelled number entry (only the opening field may be left blank)."""

    def __init__(self, parent, label: str, default: str, width: int = 12) -> None:
        super().__init__(parent)
        self.var = tk.StringVar(value=default)
        ttk.Label(self, text=label, width=30, anchor="w").pack(side="left")
        ttk.Entry(self, textvariable=self.var, width=width).pack(
            side="left", fill="x", expand=True
        )
        self.pack(fill="x", pady=3)

    def get(self) -> str:
        return self.var.get().strip()


def _req_float(field: _Field, name: str) -> float:
    raw = field.get()
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"Invalid {name}: {raw!r} is not a number.") from exc
    if not math.isfinite(value):
        raise ValueError(f"Invalid {name}: {raw!r} is not finite.")
    return value


def _opt_opening(field: _Field) -> Optional[float]:
    raw = field.get()
    if raw == "":
        return None  # use the config default
    value = _req_float(field, "opening")
    if not 0.0 <= value <= 1.0:
        raise ValueError(f"Invalid opening: must be in [0, 1]; got {value!r}.")
    return value


class RobotArmApp:
    """Main window: a notebook with the 5 workflow stages."""

    def __init__(self, root: tk.Tk, config: ra.RobotConfig = DEFAULT_CFG) -> None:
        self.root = root
        self.cfg = config
        root.title(
            f"DUM-E Robot Arm — L0={config.l0:g} L1={config.l1:g} L2={config.l2:g} "
            f"L3={config.l3:g} mm, {config.gripper_fingers}-finger gripper"
        )
        root.geometry("1000x785")
        root.minsize(900, 620)

        nb = ttk.Notebook(root)
        nb.pack(fill="both", expand=True)
        self.notebook = nb

        self._build_fk(nb)
        self._build_ik(nb)
        self._build_reach(nb)
        self._build_verify(nb)
        self._build_viz(nb)
        self.viz_tab_index = 4  # the Visualization tab is the last one

        # Animation state: last drawn pose is the FROM pose of a transition.
        self._current_angles: Optional[ra.JointAngles] = None
        self._current_target = None
        self._current_opening = None
        self._anim_frames: tuple = ()
        self._anim_limits = None
        self._anim_idx = 0
        self._anim_job = None
        self._anim_target = None
        self._anim_opening = None
        self._anim_stop_requested = False

    # ------------------------------------------------------------------
    # Shared layout for the four "compute" tabs (inputs left, output right)
    # ------------------------------------------------------------------
    def _add_compute_tab(self, nb: ttk.Notebook, title: str,
                         frame_label: str, height: int = 11):
        """Create a tab with an input ``LabelFrame`` (left) + ``_Output``
        (right); returns ``(left, out)`` so the caller fills the inputs."""
        tab = ttk.Frame(nb, padding=8)
        nb.add(tab, text=title)
        left = ttk.LabelFrame(tab, text=frame_label, padding=8)
        left.grid(row=0, column=0, sticky="ns")
        out = _Output(tab, height=height)
        out.grid(row=0, column=1, sticky="nsew", padx=8)
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(0, weight=1)
        return left, out

    def _row_buttons(self, parent, *items):
        """Pack ``(text, command)`` buttons in a row inside ``parent``."""
        row = ttk.Frame(parent)
        for text, cmd in items:
            ttk.Button(row, text=text, command=cmd).pack(side="left", padx=3)
        row.pack(fill="x", pady=6)
        return row

    # ------------------------------------------------------------------
    # Tab 1 — Forward Kinematics
    # ------------------------------------------------------------------
    def _build_fk(self, nb: ttk.Notebook) -> None:
        left, self.fk_out = self._add_compute_tab(nb, "Forward Kinematics", "Joint angles + opening")
        self.fk_base = _Field(left, "Base yaw / spin (deg)", "45.0")
        self.fk_j1 = _Field(left, "Joint 1 / L1 tilt (deg)", "30.0")
        self.fk_j2 = _Field(left, "Joint 2 / L2 tilt (deg)", "-20.0")
        self.fk_j3 = _Field(left, "Joint 3 / L3 tilt (deg)", "-60.0")
        self.fk_open = _Field(left, "opening 0..1 (blank = default)", "1.0")
        self._row_buttons(left, ("Compute FK", self._do_fk), ("Visualize", self._fk_visualize))

    def _do_fk(self) -> None:
        try:
            tb = _req_float(self.fk_base, "theta_base")
            j1 = _req_float(self.fk_j1, "joint1")
            j2 = _req_float(self.fk_j2, "joint2")
            j3 = _req_float(self.fk_j3, "joint3")
            opening = _opt_opening(self.fk_open)
            p = ra.forward_kinematics(tb, j1, j2, j3, self.cfg, opening)
            gp = ra.gripper_pose(tb, j1, j2, j3, self.cfg, opening)
        except ValueError as exc:
            self.fk_out.show(f"ERROR\n{exc}")
            return
        eff = self.cfg.gripper_opening if opening is None else opening
        lines = [
            f"Grip point      : ({p.x:9.3f}, {p.y:9.3f}, {p.z:9.3f}) mm   "
            f"(opening {eff:.3f})",
            f"Joint 1         : ({gp.joint1.x:9.3f}, {gp.joint1.y:9.3f}, {gp.joint1.z:9.3f}) mm",
            f"Joint 2         : ({gp.joint2.x:9.3f}, {gp.joint2.y:9.3f}, {gp.joint2.z:9.3f}) mm",
            f"Joint 3         : ({gp.joint3.x:9.3f}, {gp.joint3.y:9.3f}, {gp.joint3.z:9.3f}) mm",
            f"Gripper mount   : ({gp.gripper_mount.x:9.3f}, {gp.gripper_mount.y:9.3f}, {gp.gripper_mount.z:9.3f}) mm",
            f"Spread          : {gp.spread:6.3f} deg",
        ]
        for k, tip in enumerate(gp.fingertips, 1):
            lines.append(
                f"Finger {k} : ({tip.x:9.3f}, {tip.y:9.3f}, {tip.z:9.3f}) mm"
            )
        self.fk_out.show("\n".join(lines))

    def _fk_visualize(self) -> None:
        try:
            tb = _req_float(self.fk_base, "theta_base")
            j1 = _req_float(self.fk_j1, "joint1")
            j2 = _req_float(self.fk_j2, "joint2")
            j3 = _req_float(self.fk_j3, "joint3")
            opening = _opt_opening(self.fk_open)
        except ValueError as exc:
            messagebox.showerror("DUM-E", str(exc))
            return
        self._goto_viz_angles(ra.JointAngles(tb, j1, j2, j3), opening)

    # ------------------------------------------------------------------
    # Tab 2 — Inverse Kinematics
    # ------------------------------------------------------------------
    def _build_ik(self, nb: ttk.Notebook) -> None:
        left, self.ik_out = self._add_compute_tab(nb, "Inverse Kinematics", "Target grip point (mm)")

        row = ttk.Frame(left)
        ttk.Label(row, text="joint2 pose", width=30, anchor="w").pack(side="left")
        self.ik_mode = ttk.Combobox(row, values=("up", "down"), width=10, state="readonly")
        self.ik_mode.set("up")
        self.ik_mode.pack(side="left", fill="x", expand=True)
        row.pack(fill="x", pady=3)

        self.ik_x = _Field(left, "x (mm)", "300.0")
        self.ik_y = _Field(left, "y (mm)", "120.0")
        self.ik_z = _Field(left, "z (mm)", "150.0")
        row = ttk.Frame(left)
        ttk.Label(row, text="joint3 / L3 tilt (deg)", width=30, anchor="w").pack(side="left")
        self.ik_j3 = ttk.Combobox(row, values=("auto", "-60", "-55", "-50", "-45", "-40", "-35", "-30", "-25", "-20", "-15", "-10", "-5", "0", "5", "10", "15", "20", "25", "30", "35", "40", "45", "50", "55", "60", "65", "70", "75", "80", "85", "90", "95", "100", "105", "110", "115", "120", "125", "130", "135", "140", "145", "150"), width=10, state="readonly")
        self.ik_j3.set("auto")
        self.ik_j3.pack(side="left", fill="x", expand=True)
        row.pack(fill="x", pady=3)
        self.ik_open = _Field(left, "opening 0..1 (blank = default)", "1.0")
        self._row_buttons(left, ("Solve IK", self._do_ik), ("Visualize (XYZ)", self._ik_visualize))

    def _ik_inputs(self):
        x = _req_float(self.ik_x, "x")
        y = _req_float(self.ik_y, "y")
        z = _req_float(self.ik_z, "z")
        j3_str = self.ik_j3.get().strip()
        j3 = j3_str if j3_str == "auto" else float(j3_str)
        opening = _opt_opening(self.ik_open)
        joint2_up = self.ik_mode.get() != "down"
        return x, y, z, j3, opening, joint2_up

    def _do_ik(self) -> None:
        try:
            x, y, z, j3, opening, joint2_up = self._ik_inputs()
            a = ra.inverse_kinematics(x, y, z, joint2_up=joint2_up, config=self.cfg, opening=opening, joint3=j3)
        except ValueError as exc:
            self.ik_out.show(f"ERROR\n{exc}")
            return
        back = ra.forward_kinematics(a.theta_base, a.joint1, a.joint2, a.joint3, self.cfg, opening)
        err = ((back.x - x) ** 2 + (back.y - y) ** 2 + (back.z - z) ** 2) ** 0.5
        ok = ra.check_joint_limits(a.theta_base, a.joint1, a.joint2, a.joint3, self.cfg)
        self.ik_out.show(
            "IK solution (grip point = object centre):\n"
            f"  theta_base = {a.theta_base:9.3f} deg\n"
            f"  joint1     = {a.joint1:9.3f} deg\n"
            f"  joint2     = {a.joint2:9.3f} deg\n"
            f"  joint3     = {a.joint3:9.3f} deg\n"
            f"  joint2 pose: {'up' if joint2_up else 'down'}\n"
            f"  within joint limits: {'yes' if ok else 'NO - out of range'}\n"
            f"  FK(recomputed) = ({back.x:.3f}, {back.y:.3f}, {back.z:.3f}) mm\n"
            f"  round-trip error = {err:.2e} mm"
        )

    def _ik_visualize(self) -> None:
        try:
            x, y, z, j3, opening, joint2_up = self._ik_inputs()
            a = ra.inverse_kinematics(x, y, z, joint2_up=joint2_up, config=self.cfg, opening=opening, joint3=j3)
        except ValueError as exc:
            messagebox.showerror("DUM-E", str(exc))
            return
        self._goto_viz_angles(
            a, opening, target=ra.CartesianPoint(x, y, z)
        )

    # ------------------------------------------------------------------
    # Tab 3 — Reachability
    # ------------------------------------------------------------------
    def _build_reach(self, nb: ttk.Notebook) -> None:
        left, self.rc_out = self._add_compute_tab(nb, "Reachability", "Target (mm)")
        self.rc_x = _Field(left, "x (mm)", "300.0")
        self.rc_y = _Field(left, "y (mm)", "120.0")
        self.rc_z = _Field(left, "z (mm)", "150.0")
        self.rc_j3 = _Field(left, "joint3 / L3 tilt (deg)", "-60.0")
        self.rc_open = _Field(left, "opening 0..1 (blank = default)", "1.0")
        ttk.Button(left, text="Check", command=self._do_reach).pack(fill="x", pady=6)

    def _do_reach(self) -> None:
        try:
            x = _req_float(self.rc_x, "x")
            y = _req_float(self.rc_y, "y")
            z = _req_float(self.rc_z, "z")
            j3 = _req_float(self.rc_j3, "joint3")
            opening = _opt_opening(self.rc_open)
        except ValueError as exc:
            self.rc_out.show(f"ERROR\n{exc}")
            return
        eff = self.cfg.gripper_opening if opening is None else opening
        lo, hi = ra._reach_bounds_with_joint3(self.cfg, math.radians(j3), opening)
        d = (x * x + y * y + (z - self.cfg.l0) ** 2) ** 0.5
        ok = ra.is_reachable(x, y, z, self.cfg, math.radians(j3), opening)
        self.rc_out.show(
            "Reachability check:\n"
            f"  target distance from joint1 pivot d = {d:.3f} mm\n"
            f"  reach bounds (joint3={j3:.1f}, opening {eff:.3f}): [{lo:.3f}, {hi:.3f}] mm\n"
            f"  ==> {'REACHABLE (geometrically)' if ok else 'UNREACHABLE for this arm'}"
        )

    # ------------------------------------------------------------------
    # Tab 4 — Verification
    # ------------------------------------------------------------------
    def _build_verify(self, nb: ttk.Notebook) -> None:
        left, self.vf_out = self._add_compute_tab(nb, "Verification", "Suite settings", height=14)
        self.vf_n = _Field(left, "n_samples", "20")
        self.vf_open = _Field(left, "opening 0..1 (blank = default)", "1.0")
        ttk.Button(left, text="Run FK -> IK -> FK", command=self._do_verify).pack(
            fill="x", pady=6
        )

    def _do_verify(self) -> None:
        try:
            n = int(_req_float(self.vf_n, "n_samples"))
            opening = _opt_opening(self.vf_open)
        except ValueError as exc:
            self.vf_out.show(f"ERROR\n{exc}")
            return
        if n <= 0:
            self.vf_out.show("ERROR\nn_samples must be >= 1.")
            return
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            ra.verify_fk_ik(n, self.cfg, opening)
        self.vf_out.show(buf.getvalue())

    # ------------------------------------------------------------------
    # Tab 5 — Visualization
    # ------------------------------------------------------------------
    def _build_viz(self, nb: ttk.Notebook) -> None:
        tab = ttk.Frame(nb, padding=8)
        nb.add(tab, text="Visualization")

        self.viz_mode = tk.StringVar(value="angles")
        radios = ttk.Frame(tab)
        radios.pack(fill="x")
        ttk.Radiobutton(
            radios, text="From joint angles", variable=self.viz_mode,
            value="angles", command=self._toggle_viz,
        ).pack(side="left", padx=4)
        ttk.Radiobutton(
            radios, text="From XYZ target", variable=self.viz_mode,
            value="target", command=self._toggle_viz,
        ).pack(side="left", padx=4)

        # Panel A: joint angles.
        self.viz_ang = ttk.LabelFrame(tab, text="Joint angles", padding=8)
        self.viz_ang.pack(fill="x", pady=(6, 0))
        self.v_base = _Field(self.viz_ang, "Base yaw / spin (deg)", "45.0")
        self.v_j1 = _Field(self.viz_ang, "Joint 1 / L1 tilt (deg)", "30.0")
        self.v_j2 = _Field(self.viz_ang, "Joint 2 / L2 tilt (deg)", "-20.0")
        self.v_j3_ang = _Field(self.viz_ang, "Joint 3 / L3 tilt (deg)", "-60.0")
        self.v_open_a = _Field(self.viz_ang, "opening 0..1 (blank = default)", "1.0")

        # Panel B: XYZ target (hidden until selected).
        self.viz_tgt = ttk.LabelFrame(tab, text="XYZ target grip point (world frame, mm)", padding=8)
        self.v_x = _Field(self.viz_tgt, "x (mm)", "300.0")
        self.v_y = _Field(self.viz_tgt, "y (mm)", "120.0")
        self.v_z = _Field(self.viz_tgt, "z (mm)", "150.0")
        row = ttk.Frame(self.viz_tgt)
        ttk.Label(row, text="joint3 / L3 tilt (deg)", width=30, anchor="w").pack(side="left")
        self.v_j3 = ttk.Combobox(row, values=("auto", "-60", "-55", "-50", "-45", "-40", "-35", "-30", "-25", "-20", "-15", "-10", "-5", "0", "5", "10", "15", "20", "25", "30", "35", "40", "45", "50", "55", "60", "65", "70", "75", "80", "85", "90", "95", "100", "105", "110", "115", "120", "125", "130", "135", "140", "145", "150"), width=10, state="readonly")
        self.v_j3.set("auto")
        self.v_j3.pack(side="left", fill="x", expand=True)
        row.pack(fill="x", pady=3)
        self.v_open_t = _Field(self.viz_tgt, "opening 0..1 (blank = default)", "1.0")
        row = ttk.Frame(self.viz_tgt)
        ttk.Label(row, text="joint2 pose", width=30, anchor="w").pack(side="left")
        self.v_mode = ttk.Combobox(row, values=("up", "down"), width=10, state="readonly")
        self.v_mode.set("up")
        self.v_mode.pack(side="left", fill="x", expand=True)
        row.pack(fill="x", pady=3)

        btns = ttk.Frame(tab)
        self.viz_plot_btn = ttk.Button(btns, text="Plot robot", command=self._do_viz_plot)
        self.viz_plot_btn.pack(side="left", padx=4)
        ttk.Button(btns, text="Clear", command=self._viz_clear).pack(side="left", padx=4)
        self.viz_anim_btn = ttk.Button(btns, text="Animate", command=self._anim_run)
        self.viz_anim_btn.pack(side="left", padx=4)
        btns.pack(fill="x", pady=4)

        # Embedded matplotlib 3D canvas.
        container = ttk.Frame(tab)
        container.pack(fill="both", expand=True)
        self.viz_fig = Figure(figsize=(7.2, 4.4))
        self.viz_ax = self.viz_fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.viz_fig, master=container)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        toolbar = NavigationToolbar2Tk(self.canvas, container)
        toolbar.update()
        self.canvas.draw_idle()

    def _toggle_viz(self) -> None:
        if self.viz_mode.get() == "angles":
            self.viz_tgt.pack_forget()
            self.viz_ang.pack(fill="x", pady=(6, 0))
        else:
            self.viz_ang.pack_forget()
            self.viz_tgt.pack(fill="x", pady=(6, 0))

    def _resolve_viz_pose(self):
        """Turn the Visualization panels into (angles, target, opening).

        Angles mode: direct joint angles, no target.  XYZ mode: run IK on the
        target.  Raises ``ValueError`` with a user-facing message on bad input.
        """
        if self.viz_mode.get() == "angles":
            angles = ra.JointAngles(
                _req_float(self.v_base, "theta_base"),
                _req_float(self.v_j1, "joint1"),
                _req_float(self.v_j2, "joint2"),
                _req_float(self.v_j3_ang, "joint3"),
            )
            opening = _opt_opening(self.v_open_a)
            return angles, None, opening
        x = _req_float(self.v_x, "x")
        y = _req_float(self.v_y, "y")
        z = _req_float(self.v_z, "z")
        j3_str = self.v_j3.get().strip()
        j3 = j3_str if j3_str == "auto" else float(j3_str)
        opening = _opt_opening(self.v_open_t)
        joint2_up = self.v_mode.get() != "down"
        target = ra.CartesianPoint(x, y, z)
        angles = ra.inverse_kinematics(
            x, y, z, joint2_up=joint2_up, config=self.cfg, opening=opening, joint3=j3
        )
        return angles, target, opening

    def _do_viz_plot(self) -> None:
        try:
            angles, target, opening = self._resolve_viz_pose()
        except ValueError as exc:
            messagebox.showerror("DUM-E", str(exc))
            return
        self._draw(angles, target, opening)

    def _viz_clear(self) -> None:
        self.viz_ax.clear()
        self.canvas.draw_idle()

    def _draw(self, angles: ra.JointAngles, target=None, opening=None,
              fixed_limits=None) -> None:
        ra.draw_robot(self.viz_ax, angles, self.cfg, target, opening,
                      fixed_limits=fixed_limits)
        self.canvas.draw()
        self._current_angles = angles
        self._current_target = target
        self._current_opening = opening

    # ------------------------------------------------------------------
    # Smooth pose transition (Animate button)
    # ------------------------------------------------------------------
    def _anim_run(self) -> None:
        if self._anim_job is not None:  # already running -> this click stops it
            self._anim_stop()
            return
        try:
            to_angles, target, opening = self._resolve_viz_pose()
        except ValueError as exc:
            messagebox.showerror("DUM-E", str(exc))
            return
        start = self._current_angles or ra.HOME_ANGLES
        if _same_pose(start, to_angles):
            # Nothing would visibly move; draw once and tell the user why.
            self._draw(to_angles, target, opening)
            messagebox.showinfo(
                "DUM-E",
                "Start and target poses are identical - there is nothing to\n"
                "animate. Change the inputs (or enter a different target)\n"
                "and try again.",
            )
            return
        self._anim_frames = ra.interpolate_joints(start, to_angles, VIZ_ANIM_FRAMES)
        self._anim_limits = ra.animation_limits(self._anim_frames, self.cfg, opening, target)
        self._anim_idx = 0
        self._anim_target = target
        self._anim_opening = opening
        self._anim_stop_requested = False
        self.viz_anim_btn.configure(text="Stop")
        self.viz_plot_btn.configure(state="disabled")
        self._anim_step()

    def _anim_step(self) -> None:
        if self._anim_idx >= len(self._anim_frames):
            self._anim_finish()
            return
        angles = self._anim_frames[self._anim_idx]
        self._anim_idx += 1
        self._draw(angles, self._anim_target, self._anim_opening,
                   fixed_limits=self._anim_limits)
        if self._anim_idx >= len(self._anim_frames):
            self._anim_finish()  # last frame drawn; no trailing tick
        elif not self._anim_stop_requested:
            self._anim_job = self.root.after(VIZ_ANIM_INTERVAL_MS, self._anim_step)

    def _anim_stop(self) -> None:
        self._anim_stop_requested = True
        if self._anim_job is not None:
            self.root.after_cancel(self._anim_job)
            self._anim_job = None
        self._anim_finish()

    def _anim_finish(self) -> None:
        self._anim_job = None
        self.viz_anim_btn.configure(text="Animate")
        self.viz_plot_btn.configure(state="normal")

    def _goto_viz_angles(self, angles: ra.JointAngles, opening=None, target=None) -> None:
        # Switch to the Visualization tab, pre-fill the joint-angle panel and draw.
        self.viz_mode.set("angles")
        self._toggle_viz()
        self.v_base.var.set(f"{angles.theta_base:.3f}")
        self.v_j1.var.set(f"{angles.joint1:.3f}")
        self.v_j2.var.set(f"{angles.joint2:.3f}")
        self.v_open_a.var.set("" if opening is None else f"{opening:.3f}")
        self.notebook.select(self.viz_tab_index)
        self._draw(angles, target, opening)


def main() -> None:
    root = tk.Tk()
    RobotArmApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()