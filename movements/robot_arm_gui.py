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
import json
import math
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Optional

import robot_arm as ra

import matplotlib

try:
    matplotlib.use("TkAgg")  # select the GUI backend before importing its canvas
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

# Wall-clock paced animation. More samples make the motion smoother without
# making it slower: a slow redraw skips frames instead of stretching the move.
VIZ_ANIM_FRAMES = ra.VIZ_ANIM_FRAMES
VIZ_ANIM_FRAME_MS = ra.VIZ_ANIM_FRAME_MS
JOINT3_CHOICES = ("auto", "down") + tuple(str(v) for v in range(-60, 151, 5))

# Compact, consistent widget metrics shared by the editable joint panels.
UI_FONT = ("Segoe UI", 9)
VALUE_FONT = ("Consolas", 9)
JOINT_LABEL_WIDTH = 10
JOINT_ENTRY_WIDTH = 8
UI_PAD_X = 8
UI_PAD_Y = 4

BASE_DIR = Path(__file__).resolve().parent.parent
MIND_STATE_FILE = BASE_DIR / "mind" / "robot_state.json"
ROOT_STATE_FILE = BASE_DIR / "robot_state.json"


def _write_state(path: Path, state: dict) -> None:
    temp = path.with_name(path.name + ".tmp")
    with temp.open("w", encoding="utf-8") as fp:
        json.dump(state, fp, indent=2)
        fp.write("\n")
        fp.flush()
    temp.replace(path)


def _persist_pose(angles: ra.JointAngles, opening: Optional[float],
                  config: ra.RobotConfig) -> None:
    effective_opening = config.gripper_opening if opening is None else opening
    state = {
        "base": angles.theta_base,
        "joint1": angles.joint1,
        "joint2": angles.joint2,
        "joint3": angles.joint3,
        "gripper_roll": angles.gripper_roll,
        "gripper": 100.0 * effective_opening,
    }
    _write_state(MIND_STATE_FILE, state)
    _write_state(ROOT_STATE_FILE, state)


def _same_pose(a: ra.JointAngles, b: ra.JointAngles, tol: float = 1e-3) -> bool:
    """True when two poses are (numerically) the same joint position."""
    return all(
        abs(getattr(a, f) - getattr(b, f)) <= tol
        for f in ("theta_base", "joint1", "joint2", "joint3", "gripper_roll")
    )


def _same_opening(a: float | None, b: float | None, tol: float = 1e-6) -> bool:
    """True when two grip openings are (numerically) equal (None == None)."""
    if a is None or b is None:
        return a is b
    return abs(a - b) <= tol


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

    def __init__(self, parent, label: str, default: str, width: int = 10,
                 pack: bool = True, label_width: int = 18,
                 font: Optional[tuple] = None, expand: bool = True) -> None:
        super().__init__(parent)
        self.var = tk.StringVar(value=default)
        ttk.Label(self, text=label, width=label_width, anchor="w",
                  font=font or UI_FONT).pack(side="left", padx=(0, 4))
        self.entry = ttk.Entry(self, textvariable=self.var, width=width,
                               font=font or UI_FONT)
        self.entry.pack(side="left", fill="x", expand=expand)
        if pack:
            self.pack(fill="x", pady=2)

    def get(self) -> str:
        return self.var.get().strip()


class _SpinField(_Field):
    """A compact labelled spinbox constrained to a joint's configured range."""

    def __init__(self, parent, label: str, default: str,
                 limits: tuple[float, float], width: int = JOINT_ENTRY_WIDTH,
                 label_width: int = JOINT_LABEL_WIDTH) -> None:
        super().__init__(parent, label, default, width=width, pack=False,
                         label_width=label_width, font=VALUE_FONT, expand=False)
        self.entry.destroy()
        low, high = limits
        increment = 0.05 if high <= 1.0 else 1.0
        self.entry = ttk.Spinbox(
            self, from_=low, to=high, increment=increment,
            textvariable=self.var, width=width, font=VALUE_FONT,
        )
        self.entry.pack(side="left")


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
        root.geometry("960x720")
        root.minsize(860, 600)

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
        self._anim_openings: tuple = ()
        self._anim_limits = None
        self._anim_idx = 0
        self._anim_job = None
        self._anim_start = time.perf_counter()
        self._anim_tick = 1
        self._anim_target = None
        self._anim_opening = None
        self._anim_stop_requested = False

    # ------------------------------------------------------------------
    # Compact field placement shared by every tab
    # ------------------------------------------------------------------
    @staticmethod
    def _grid_field(parent, name: str, label: str, default: str,
                    row: int, column: int, owner) -> None:
        field = _Field(parent, label, default, pack=False)
        field.grid(row=row, column=column, sticky="ew",
                   padx=(0, UI_PAD_X), pady=UI_PAD_Y)
        setattr(owner, name, field)

    @staticmethod
    def _grid_joint_field(parent, name: str, label: str, default: str,
                          limits: tuple[float, float], row: int, column: int,
                          owner) -> None:
        field = _SpinField(parent, label, default, limits)
        field.grid(row=row, column=column, sticky="w",
                   padx=(0, UI_PAD_X), pady=UI_PAD_Y)
        setattr(owner, name, field)

    @staticmethod
    def _grid_combo(parent, name: str, label: str, values, default: str,
                    row: int, column: int, owner, width: int = 10) -> None:
        cell = ttk.Frame(parent)
        cell.grid(row=row, column=column, sticky="ew",
                  padx=(0, UI_PAD_X), pady=UI_PAD_Y)
        ttk.Label(cell, text=label, width=18, anchor="w",
                  font=UI_FONT).pack(side="left")
        combo = ttk.Combobox(cell, values=values, width=width,
                             state="readonly", font=UI_FONT)
        combo.set(default)
        combo.pack(side="left", fill="x", expand=True)
        setattr(owner, name, combo)

    @staticmethod
    def _two_columns(parent) -> None:
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)

    def _joint_values(self, parent, prefix: str, base="45.0", j1="30.0",
                      j2="-20.0", j3="-60.0", roll="0.0", opening="1.0",
                      j3_name=None, j4_name=None, open_name=None,
                      roll_alias=None, columns: int = 2):
        """Return one aligned grid of editable joint values.

        The compute/viz panes are narrow, so they default to two columns;
        the full-width Dum-E strip uses three.
        """
        grid = ttk.Frame(parent)
        for column in range(columns):
            grid.columnconfigure(column, weight=1, uniform=f"joint{columns}")

        j3_name = j3_name or f"{prefix}_j3"
        j4_name = j4_name or f"{prefix}_j4"
        open_name = open_name or f"{prefix}_open"
        roll_alias = roll_alias or f"{prefix}_roll"
        fields = (
            (f"{prefix}_base", "Base yaw", base, "base"),
            (f"{prefix}_j1", "J1 tilt", j1, "joint1"),
            (f"{prefix}_j2", "J2 tilt", j2, "joint2"),
            (j3_name, "J3 tilt", j3, "joint3"),
            (j4_name, "J4 roll", roll, "joint4"),
            (open_name, "Grip 0..1", opening, "opening"),
        )
        for index, (name, label, default, key) in enumerate(fields):
            limits = (
                (0.0, 1.0) if key == "opening"
                else self.cfg.joint_limits[key]
            )
            self._grid_joint_field(grid, name, label, default, limits,
                                   index // columns, index % columns, self)
        setattr(self, roll_alias, getattr(self, j4_name))
        return grid

    # ------------------------------------------------------------------
    # Shared layout for the four "compute" tabs (inputs left, output right)
    # ------------------------------------------------------------------
    def _add_compute_tab(self, nb: ttk.Notebook, title: str,
                         frame_label: str, height: int = 11):
        """Create a tab with an input ``LabelFrame`` (left) + ``_Output``
        (right); returns ``(left, out)`` so the caller fills the inputs."""
        tab = ttk.Frame(nb, padding=8)
        nb.add(tab, text=title)
        left = ttk.LabelFrame(tab, text=frame_label, padding=6)
        left.grid(row=0, column=0, sticky="ns")
        out = _Output(tab, height=height)
        out.grid(row=0, column=1, sticky="nsew", padx=8)
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(0, weight=1)
        return left, out

    def _row_buttons(self, parent, row: int, *items):
        """Grid ``(text, command)`` buttons in a row inside ``parent``."""
        button_row = ttk.Frame(parent)
        for text, cmd in items:
            ttk.Button(button_row, text=text, command=cmd).pack(side="left", padx=3)
        button_row.grid(row=row, column=0, columnspan=2, sticky="ew", pady=(6, 2))
        return button_row

    # ------------------------------------------------------------------
    # Tab 1 — Forward Kinematics
    # ------------------------------------------------------------------
    def _build_fk(self, nb: ttk.Notebook) -> None:
        left, self.fk_out = self._add_compute_tab(nb, "Forward Kinematics", "Joint angles + opening")
        self._two_columns(left)
        fields = self._joint_values(left, "fk")
        fields.grid(row=0, column=0, columnspan=2, sticky="ew")
        self._row_buttons(left, 1, ("Compute FK", self._do_fk), ("Visualize", self._fk_visualize))

    def _do_fk(self) -> None:
        try:
            tb = _req_float(self.fk_base, "theta_base")
            j1 = _req_float(self.fk_j1, "joint1")
            j2 = _req_float(self.fk_j2, "joint2")
            j3 = _req_float(self.fk_j3, "joint3")
            roll = _req_float(self.fk_j4, "joint4")
            opening = _opt_opening(self.fk_open)
            p = ra.forward_kinematics(tb, j1, j2, j3, self.cfg, opening, roll)
            gp = ra.gripper_pose(tb, j1, j2, j3, self.cfg, opening, roll)
        except ValueError as exc:
            self.fk_out.show(f"ERROR\n{exc}")
            return
        _persist_pose(ra.JointAngles(tb, j1, j2, j3, roll), opening, self.cfg)
        eff = self.cfg.gripper_opening if opening is None else opening
        lines = [
            f"Grip point      : ({p.x:9.3f}, {p.y:9.3f}, {p.z:9.3f}) mm   "
            f"(opening {eff:.3f})",
            f"Joint 1         : ({gp.joint1.x:9.3f}, {gp.joint1.y:9.3f}, {gp.joint1.z:9.3f}) mm",
            f"Joint 2         : ({gp.joint2.x:9.3f}, {gp.joint2.y:9.3f}, {gp.joint2.z:9.3f}) mm",
            f"Joint 3         : ({gp.joint3.x:9.3f}, {gp.joint3.y:9.3f}, {gp.joint3.z:9.3f}) mm",
            f"Gripper mount   : ({gp.gripper_mount.x:9.3f}, {gp.gripper_mount.y:9.3f}, {gp.gripper_mount.z:9.3f}) mm",
            f"Spread          : {gp.spread:6.3f} deg",
            f"J4 wrist roll   : {gp.joint4:6.3f} deg",
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
            roll = _req_float(self.fk_j4, "joint4")
            opening = _opt_opening(self.fk_open)
        except ValueError as exc:
            messagebox.showerror("DUM-E", str(exc))
            return
        self._goto_viz_angles(ra.JointAngles(tb, j1, j2, j3, roll), opening)

    # ------------------------------------------------------------------
    # Tab 2 — Inverse Kinematics
    # ------------------------------------------------------------------
    def _build_ik(self, nb: ttk.Notebook) -> None:
        left, self.ik_out = self._add_compute_tab(nb, "Inverse Kinematics", "Target grip point (mm)")
        self._two_columns(left)
        self._grid_field(left, "ik_x", "x (mm)", "300.0", 0, 0, self)
        self._grid_field(left, "ik_y", "y (mm)", "120.0", 0, 1, self)
        self._grid_field(left, "ik_z", "z (mm)", "150.0", 1, 0, self)
        self._grid_field(left, "ik_open", "Opening 0..1", "1.0", 1, 1, self)
        self._grid_field(left, "ik_roll", "J4 wrist roll", "", 2, 0, self)
        self._grid_combo(left, "ik_j3", "Joint 3", JOINT3_CHOICES, "auto", 2, 1, self)
        self._grid_combo(left, "ik_mode", "Joint 2 pose", ("up", "down"), "up", 3, 0, self)
        self._row_buttons(left, 4, ("Solve IK", self._do_ik), ("Visualize (XYZ)", self._ik_visualize))

    def _ik_inputs(self):
        x = _req_float(self.ik_x, "x")
        y = _req_float(self.ik_y, "y")
        z = _req_float(self.ik_z, "z")
        j3_str = self.ik_j3.get().strip()
        j3 = ra._parse_joint3_mode(j3_str)
        opening = _opt_opening(self.ik_open)
        roll_text = self.ik_roll.get().strip()
        roll = (
            float(roll_text) if roll_text
            else (self._current_angles.joint4 if self._current_angles else 0.0)
        )
        joint2_up = self.ik_mode.get() != "down"
        return x, y, z, j3, opening, joint2_up, roll

    def _do_ik(self) -> None:
        try:
            x, y, z, j3, opening, joint2_up, roll = self._ik_inputs()
            a = ra.inverse_kinematics(
                x, y, z, joint2_up=joint2_up, config=self.cfg, opening=opening,
                joint3=j3, gripper_roll=roll,
            )
        except ValueError as exc:
            self.ik_out.show(f"ERROR\n{exc}")
            return
        _persist_pose(a, opening, self.cfg)
        back = ra.forward_kinematics(a.theta_base, a.joint1, a.joint2, a.joint3, self.cfg, opening)
        err = ((back.x - x) ** 2 + (back.y - y) ** 2 + (back.z - z) ** 2) ** 0.5
        ok = ra.check_joint_limits(
            a.theta_base, a.joint1, a.joint2, a.joint3, self.cfg, a.gripper_roll
        )
        self.ik_out.show(
            "IK solution (grip point = object centre):\n"
            f"  theta_base = {a.theta_base:9.3f} deg\n"
            f"  joint1     = {a.joint1:9.3f} deg\n"
            f"  joint2     = {a.joint2:9.3f} deg\n"
            f"  joint3     = {a.joint3:9.3f} deg\n"
            f"  joint4 (wrist roll) = {a.joint4:7.3f} deg\n"
            f"  joint2 pose: {'up' if joint2_up else 'down'}\n"
            f"  within joint limits: {'yes' if ok else 'NO - out of range'}\n"
            f"  FK(recomputed) = ({back.x:.3f}, {back.y:.3f}, {back.z:.3f}) mm\n"
            f"  round-trip error = {err:.2e} mm"
        )

    def _ik_visualize(self) -> None:
        try:
            x, y, z, j3, opening, joint2_up, roll = self._ik_inputs()
            a = ra.inverse_kinematics(
                x, y, z, joint2_up=joint2_up, config=self.cfg, opening=opening,
                joint3=j3, gripper_roll=roll,
            )
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
        self._two_columns(left)
        self._grid_field(left, "rc_x", "x (mm)", "300.0", 0, 0, self)
        self._grid_field(left, "rc_y", "y (mm)", "120.0", 0, 1, self)
        self._grid_field(left, "rc_z", "z (mm)", "150.0", 1, 0, self)
        self._grid_field(left, "rc_j3", "Joint 3 tilt", "-60.0", 1, 1, self)
        self._grid_field(left, "rc_open", "Opening 0..1", "1.0", 2, 0, self)
        ttk.Button(left, text="Check", command=self._do_reach).grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(6, 2)
        )

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
        self._two_columns(left)
        self._grid_field(left, "vf_n", "Samples", "20", 0, 0, self)
        self._grid_field(left, "vf_open", "Opening 0..1", "1.0", 0, 1, self)
        ttk.Button(left, text="Run FK -> IK -> FK", command=self._do_verify).grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=(6, 2)
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
        tab = ttk.Frame(nb, padding=6)
        nb.add(tab, text="Visualization")

        # Mode choices and actions share one strip; the canvas gets the rest.
        header = ttk.Frame(tab)
        header.pack(fill="x")
        self.viz_mode = tk.StringVar(value="angles")
        radios = ttk.Frame(header)
        radios.pack(side="left")
        ttk.Radiobutton(
            radios, text="Joint angles", variable=self.viz_mode,
            value="angles", command=self._toggle_viz,
        ).pack(side="left", padx=(0, 6))
        ttk.Radiobutton(
            radios, text="XYZ target", variable=self.viz_mode,
            value="target", command=self._toggle_viz,
        ).pack(side="left", padx=(0, 6))
        buttons = ttk.Frame(header)
        buttons.pack(side="right")
        self.viz_plot_btn = ttk.Button(buttons, text="Plot", command=self._do_viz_plot)
        self.viz_plot_btn.pack(side="left", padx=2)
        ttk.Button(buttons, text="Clear", command=self._viz_clear).pack(side="left", padx=2)
        self.viz_anim_btn = ttk.Button(buttons, text="Animate", command=self._anim_run)
        self.viz_anim_btn.pack(side="left", padx=2)

        panes = ttk.Panedwindow(tab, orient="horizontal")
        panes.pack(fill="both", expand=True, pady=(4, 0))
        controls = ttk.Frame(panes, padding=(0, 0, 8, 0))
        container = ttk.Frame(panes)
        panes.add(controls, weight=1)
        panes.add(container, weight=4)

        # Panel A: joint angles.
        self.viz_ang = ttk.LabelFrame(controls, text="Joint values", padding=6)
        self._two_columns(self.viz_ang)
        fields = self._joint_values(
            self.viz_ang, "v",
            j3_name="v_j3_ang", j4_name="v_j4_ang",
            open_name="v_open_a", roll_alias="v_roll_ang",
        )
        fields.grid(row=0, column=0, columnspan=2, sticky="ew")
        self.viz_ang.pack(fill="x", pady=(4, 0))

        # Panel B: XYZ target (hidden until selected).
        self.viz_tgt = ttk.LabelFrame(controls, text="XYZ target (mm)", padding=6)
        self._two_columns(self.viz_tgt)
        self._grid_field(self.viz_tgt, "v_x", "x (mm)", "300.0", 0, 0, self)
        self._grid_field(self.viz_tgt, "v_y", "y (mm)", "120.0", 0, 1, self)
        self._grid_field(self.viz_tgt, "v_z", "z (mm)", "150.0", 1, 0, self)
        self._grid_combo(self.viz_tgt, "v_j3", "Joint 3", JOINT3_CHOICES,
                         "auto", 1, 1, self)
        self._grid_field(self.viz_tgt, "v_open_t", "Opening 0..1", "1.0", 2, 0, self)
        self._grid_field(self.viz_tgt, "v_roll_t", "J4 wrist roll", "", 2, 1, self)
        self._grid_combo(self.viz_tgt, "v_mode", "Joint 2 pose", ("up", "down"),
                         "up", 3, 0, self)

        # Embedded matplotlib 3D canvas.
        self.viz_fig = Figure(figsize=(7.2, 4.4))
        self.viz_ax = self.viz_fig.add_subplot(111, projection="3d")
        self.viz_fig.subplots_adjust(left=0.03, right=0.985,
                                     bottom=0.03, top=0.97)
        self.viz_view = ra.RobotView(self.viz_ax, self.cfg)
        self.canvas = FigureCanvasTkAgg(self.viz_fig, master=container)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        toolbar = NavigationToolbar2Tk(self.canvas, container)
        toolbar.update()
        self.canvas.draw_idle()
        self._toggle_viz()

    def _toggle_viz(self) -> None:
        if self.viz_mode.get() == "angles":
            self.viz_tgt.pack_forget()
            self.viz_ang.pack(fill="x", pady=(4, 0))
        else:
            self.viz_ang.pack_forget()
            self.viz_tgt.pack(fill="x", pady=(4, 0))

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
                _req_float(self.v_j4_ang, "joint4"),
            )
            opening = _opt_opening(self.v_open_a)
            return angles, None, opening
        x = _req_float(self.v_x, "x")
        y = _req_float(self.v_y, "y")
        z = _req_float(self.v_z, "z")
        j3_str = self.v_j3.get().strip()
        j3 = ra._parse_joint3_mode(j3_str)
        opening = _opt_opening(self.v_open_t)
        roll_text = self.v_roll_t.get().strip()
        roll = (
            float(roll_text) if roll_text
            else (self._current_angles.joint4 if self._current_angles else 0.0)
        )
        joint2_up = self.v_mode.get() != "down"
        target = ra.CartesianPoint(x, y, z)
        angles = ra.inverse_kinematics(
            x, y, z, joint2_up=joint2_up, config=self.cfg, opening=opening,
            joint3=j3, gripper_roll=roll,
        )
        return angles, target, opening

    def _do_viz_plot(self) -> None:
        try:
            angles, target, opening = self._resolve_viz_pose()
        except ValueError as exc:
            messagebox.showerror("DUM-E", str(exc))
            return
        _persist_pose(angles, opening, self.cfg)
        self._draw(angles, target, opening)

    def _viz_clear(self) -> None:
        view = getattr(self, "viz_view", None)
        if view is not None:
            view.clear()
        else:  # defensive: a partially built app should still clear the axes
            self.viz_ax.clear()
        self.canvas.draw_idle()

    def _draw(self, angles: ra.JointAngles, target=None, opening=None,
              fixed_limits=None) -> None:
        view = getattr(self, "viz_view", None)
        if view is not None:
            view.draw(angles, target, opening, fixed_limits)
        else:
            ra.draw_robot(self.viz_ax, angles, self.cfg, target, opening,
                          fixed_limits=fixed_limits)
        self.canvas.draw_idle()
        self._current_angles = angles
        self._current_target = target
        self._current_opening = opening

    # ------------------------------------------------------------------
    # Smooth pose transition (Animate button)
    # ------------------------------------------------------------------
    def _cancel_anim_job(self) -> None:
        """Drop a pending frame so a new move cannot leave two timers running."""
        if self._anim_job is not None:
            try:
                self.root.after_cancel(self._anim_job)
            except tk.TclError:
                pass
            self._anim_job = None

    def _start_animation(self, goal: ra.JointAngles, opening: Optional[float],
                         target, from_opening: Optional[float],
                         start: Optional[ra.JointAngles] = None) -> None:
        """Prepare frames and run them on a fixed wall-clock timeline.

        If a redraw is slow, the next tick jumps to the frame that is actually
        due instead of queuing every intermediate pose. The move therefore
        keeps the same duration no matter how fast the canvas draws.
        """
        self._cancel_anim_job()
        target_opening = self.cfg.gripper_opening if opening is None else opening
        if from_opening is None:
            from_opening = target_opening
        start_angles = start if start is not None else (self._current_angles or goal)
        self._anim_frames = ra.interpolate_joints(start_angles, goal)
        self._anim_openings = ra.interpolate_opening(
            from_opening, target_opening, len(self._anim_frames)
        )
        self._anim_limits = ra.animation_limits(
            self._anim_frames, self.cfg, self._anim_openings, target
        )
        self._anim_idx = 0
        self._anim_target = target
        self._anim_opening = target_opening
        self._anim_stop_requested = False
        self._anim_start = time.perf_counter()
        self._anim_tick = 1
        self.viz_anim_btn.configure(text="Stop")
        self.viz_plot_btn.configure(state="disabled")
        self._anim_step()

    def _anim_run(self) -> None:
        if self._anim_job is not None:  # already running -> this click stops it
            self._anim_stop()
            return
        try:
            goal, target, opening = self._resolve_viz_pose()
        except ValueError as exc:
            messagebox.showerror("DUM-E", str(exc))
            return
        # Save the commanded pose before the first animation frame begins.
        _persist_pose(goal, opening, self.cfg)
        start = self._current_angles or ra.HOME_ANGLES
        start_opening = (self._current_opening if self._current_opening is not None
                         else self.cfg.gripper_opening)
        target_opening = self.cfg.gripper_opening if opening is None else opening
        if _same_pose(start, goal) and _same_opening(start_opening, target_opening):
            # Nothing would visibly move; draw once and tell the user why.
            self._draw(goal, target, opening)
            messagebox.showinfo(
                "DUM-E",
                "Start and target poses are identical - there is nothing to\n"
                "animate. Change the inputs (or enter a different target)\n"
                "and try again.",
            )
            return
        self._start_animation(goal, opening, target, start_opening, start=start)

    def _anim_step(self) -> None:
        if not self._anim_frames or self._anim_stop_requested:
            self._anim_finish()
            return
        frame_count = len(self._anim_frames)
        duration_ms = (frame_count - 1) * VIZ_ANIM_FRAME_MS
        elapsed_ms = (time.perf_counter() - self._anim_start) * 1000.0
        progress = min(1.0, elapsed_ms / duration_ms) if duration_ms > 0 else 1.0
        index = min(int(progress * (frame_count - 1)), frame_count - 1)
        self._anim_idx = index + 1
        self._draw(self._anim_frames[index], self._anim_target,
                   self._anim_openings[index], self._anim_limits)
        if index >= frame_count - 1:
            self._anim_finish()
            return

        # Aim at the next absolute tick, not "N ms after this draw finished".
        self._anim_tick = max(self._anim_tick, index + 1)
        next_at = self._anim_start + self._anim_tick * VIZ_ANIM_FRAME_MS / 1000.0
        delay_ms = max(0.0, (next_at - time.perf_counter()) * 1000.0)
        if self._anim_stop_requested:
            return
        try:
            self._anim_job = self.root.after(int(delay_ms), self._anim_step)
        except tk.TclError:
            self._anim_finish()

    def _anim_stop(self) -> None:
        self._anim_stop_requested = True
        self._cancel_anim_job()
        self._anim_finish()

    def _anim_finish(self) -> None:
        self._anim_job = None
        canvas = getattr(self, "canvas", None)
        if canvas is not None:
            canvas.draw_idle()
        self.viz_anim_btn.configure(text="Animate")
        self.viz_plot_btn.configure(state="normal")

    def _goto_viz_angles(self, angles: ra.JointAngles, opening=None, target=None) -> None:
        # Switch to the Visualization tab, pre-fill the joint-angle panel and draw.
        self.viz_mode.set("angles")
        self._toggle_viz()
        self.v_base.var.set(f"{angles.theta_base:.3f}")
        self.v_j1.var.set(f"{angles.joint1:.3f}")
        self.v_j2.var.set(f"{angles.joint2:.3f}")
        self.v_j3_ang.var.set(f"{angles.joint3:.3f}")
        roll_field = getattr(self, "v_j4_ang", self.v_roll_ang)
        roll_field.var.set(f"{angles.joint4:.3f}")
        self.v_open_a.var.set("" if opening is None else f"{opening:.3f}")
        self.notebook.select(self.viz_tab_index)
        _persist_pose(angles, opening, getattr(self, "cfg", DEFAULT_CFG))
        self._draw(angles, target, opening)


def main() -> None:
    root = tk.Tk()
    RobotArmApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()