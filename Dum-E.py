"""
DUM-E - a natural-language robot arm with its GUI and kinematics in one file.

This merges three previously separate programs:

    mind/mind.py           shared command parser + local Qwen3 model
    movements/robot_arm.py forward/inverse kinematics, reachability, drawing
    movements/robot_arm_gui.py  the five-stage Tkinter workflow

Everything now shares ONE angle contract, the one the trained model was fitted
against and that ``robot_state.json`` already stores:

    base  -180 .. 180   yaw about +Z, 0 = +X, 90 = +Y, ±180 = -X
    joint1 -90 .. 90    shoulder angle from horizontal outward (positive rises)
    joint2 -150 .. 150   link L2 RELATIVE to L1
    joint3 -120 .. 120   link L3 RELATIVE to L2
    joint4 -180 .. 180   wrist/tool roll about the tool axis (legacy gripper_roll)
    gripper   0 .. 100   0 = closed, 100 = open

The link chain is base -> fixed vertical L0 -> joint1 -> L1 -> joint2 -> L2
-> joint3 -> L3 -> gripper mount.  ``joint1 + joint2 + joint3`` is the absolute
tool elevation, so the three angles CUMULATE down the arm; the geometry below
converts to the per-link absolutes the renderer needs.

Run it with:

    python Dum-E.py

Run the symmetrical two-finger version with:

    python Dum-E-2finger.py

It shares ``mind/robot_command.json`` and ``mind/robot_state.json`` with
``mind/mind.py``, so the CLI and this window drive the same arm and remember the
same pose.

The Command tab is the brain: type plain English, and if the deterministic
parser is not certain it hands the request to the fine-tuned model that sits in
``mind/dum-e-qwen3-merged``.  Nothing here talks to Ollama.
"""

from __future__ import annotations

import json
import math
import queue
import random
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Any, Iterable, Optional

import numpy as np

from robot_contract import JOINT_LIMITS

BASE_DIR = Path(__file__).resolve().parent
MIND_DIR = BASE_DIR / "mind"
sys.path.insert(0, str(MIND_DIR))
from mind import (
    ALL_JOINTS,
    DEFAULT_STATE,
    HELP_TEXT,
    JOINT_RANGES,
    MOVE_ACTIONS,
    SYSTEM_PROMPT,
    build_local_command,
    coerce_action,
    is_valid_command,
    local_overrides,
    parse_intent,
    resolve_command,
)
from mind import _in_range, _number

MODEL_PATH = MIND_DIR / "dum-e-qwen3-merged"
# Shared with mind/mind.py and the previous GUI, so whichever program the
# hardware is watching, both brains write the same command and the same pose.
COMMAND_FILE = MIND_DIR / "robot_command.json"
STATE_FILE = MIND_DIR / "robot_state.json"
ROOT_STATE_FILE = BASE_DIR / "robot_state.json"
PROMPT_FILE = MIND_DIR / "system_prompt.txt"

# =====================================================================
# 1. THE ANGLE CONTRACT
# =====================================================================

# JointAngles field name -> the key it has in JOINT_RANGES.  Only the base yaw
# is spelled differently, because the dataclass inherited the old name.
ANGLE_JOINTS = (("theta_base", "base"), ("joint1", "joint1"),
                ("joint2", "joint2"), ("joint3", "joint3"),
                ("joint4", "joint4"))

# The side panel shows each physical control once: joint4 replaces the legacy
# gripper_roll row, and gripper is the opening percentage.
POSE_KEYS = ("base", "joint1", "joint2", "joint3", "joint4", "gripper")
POSE_LABELS = {
    "base": "Base", "joint1": "J1", "joint2": "J2", "joint3": "J3",
    "joint4": "J4", "gripper": "Grip",
}

# Compact, consistent widget metrics shared by the editable joint panels.
UI_FONT = ("Segoe UI", 9)
VALUE_FONT = ("Consolas", 9)
JOINT_LABEL_WIDTH = 10
JOINT_ENTRY_WIDTH = 8
UI_PAD_X = 8
UI_PAD_Y = 4
PANEL_TOP_PAD = 12
JOINT_TOP_PAD = 8

# Link lengths in millimetres and the 3-finger end effector.
L0 = 100.0
L1 = 400.0
L2 = 300.0
L3 = 250.0
FINGER_COUNT = 3
FINGER_LENGTH = 150.0  # Lf: arc length of one rigid finger [mm]
FINGER_SPREAD_DEG = 30.0
# A finger is one RIGID curved bar (a talon), not a straight segment and not a
# hinged chain: its curvature never changes, and `opening` only swings the whole
# piece about its base.  FINGER_CURL_DEG is how far it curls inward.
FINGER_CURL_DEG = 60.0
# Finger base ring radius, sized so a fully closed finger's tip lands exactly
# on the tool axis, letting a curled finger close *onto* the axis.
#   R_arc  = FINGER_LENGTH / radians(FINGER_CURL_DEG)
#   R_base = R_arc * (1 - cos(FINGER_CURL_DEG))
FINGER_ARC_RADIUS = FINGER_LENGTH / math.radians(FINGER_CURL_DEG)
FINGER_BASE_RADIUS = FINGER_ARC_RADIUS * (1.0 - math.cos(math.radians(FINGER_CURL_DEG)))
# On-axis grip depth, a CONSTANT: the grasp point is the object centre, which
# does not move when the fingers swing.  Deriving it from the finger angle
# instead (as finger_length*cos(spread) once did) leaked the opening into the
# kinematic chain, so one target solved to different joints at different
# openings.  Value = R_arc * sin(FINGER_CURL_DEG), the depth at which the tips
# meet the axis when closed.
GRIP_DEPTH = FINGER_ARC_RADIUS * math.sin(math.radians(FINGER_CURL_DEG))
# Sampled centreline of each rigid finger, for drawing the claw as a curve.
FINGER_PATH_SAMPLES = 12
# Where the tool axis sits relative to L2 when the wrist is at rest; used to
# prefer a natural-looking IK solution over a contorted one.
WRIST_HOME = -60.0
# Absolute stem orientation for the ``joint3="down"`` IK mode: the L3 stem plus
# the gripper grip depth points straight down in the WORLD frame, so
# ``joint1 + joint2 + joint3 == STEM_DOWN_ANGLE_DEG`` for any reachable pose.
# Distinct from WRIST_HOME, which is a wrist angle RELATIVE to L2.
STEM_DOWN_ANGLE_DEG = -90.0

# Animation runs at a fixed wall-clock pace. More samples make the motion
# smoother without making it slower: a slow redraw simply skips intermediate
# frames instead of stretching the whole move.
VIZ_ANIM_FRAMES = 60
VIZ_ANIM_FRAME_MS = 16
# How often the Command tab drains finished model requests off the worker thread.
POLL_INTERVAL_MS = 60
MODEL_REQUEST_QUEUE_SIZE = 1

MODEL: Any = None
TOKENIZER: Any = None
MODEL_LOCK = threading.Lock()


# =====================================================================
# 2. REMEMBERED POSE AND THE COMMAND FILE
# =====================================================================

def _write_json(path: Path, data: dict) -> None:
    """Write via a sibling temp file and rename, so a crash or a full disk can
    never leave a half-written robot_state.json behind."""
    temp = path.with_name(path.name + ".tmp")
    with temp.open("w", encoding="utf-8") as fp:
        json.dump(data, fp, indent=2)
        fp.write("\n")
        fp.flush()
    temp.replace(path)


def _write_state_files(state: dict) -> None:
    """Keep the shared mind state and root-level GUI state mirror in sync."""
    _write_json(STATE_FILE, state)
    _write_json(ROOT_STATE_FILE, state)


def load_state() -> dict:
    """The last pose Dum-E remembers, per joint.  Anything unusable falls back
    to the default for that joint alone, so one bad value never erases the
    whole remembered pose."""
    try:
        stored = json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return DEFAULT_STATE.copy()
    if not isinstance(stored, dict):
        return DEFAULT_STATE.copy()
    state = DEFAULT_STATE.copy()
    for joint in ALL_JOINTS:
        value = stored.get(joint)
        if _in_range(value, *JOINT_RANGES[joint]):
            state[joint] = value
    return state


def ensure_state_file() -> dict:
    """Guarantee robot_state.json exists and holds five concrete angles.  On a
    first run this creates it at the default pose.  An angle that is missing,
    null or out of range is replaced by its default; every angle that is still
    usable is kept, so this never throws away what the arm has been doing."""
    state = load_state()
    _write_state_files(state)
    return state


def save_state(command: dict) -> None:
    """Fold a command into the remembered pose.  A null or unusable angle keeps
    the remembered one, so the pose on disk only ever moves to a value the arm
    can actually hold."""
    if command["action"] == "home":
        _write_state_files(DEFAULT_STATE.copy())
        return
    if command["action"] not in MOVE_ACTIONS:
        return
    state = load_state()
    for joint in ALL_JOINTS:
        value = command.get(joint)
        if _in_range(value, *JOINT_RANGES[joint]):
            state[joint] = value
    _write_state_files(state)


def save_pose_state(pose: dict) -> None:
    """Persist a complete, validated pose produced by a GUI visualization mode."""
    state = {}
    for joint in ALL_JOINTS:
        value = pose.get(joint)
        if not _in_range(value, *JOINT_RANGES[joint]):
            raise ValueError(f"Invalid {joint} angle {value!r}; refusing to save pose.")
        state[joint] = _number(value)
    _write_state_files(state)


def absolute_command(command: dict) -> dict:
    """The command with every unspecified angle filled in from memory.

    A resolver command only names the joints it actually moves, so the rest
    arrive as null. The arm still has to be told where those joints are, so
    each one takes its remembered value here. That makes the emitted pose the
    full post-command pose: an angle the command names is the new one, and an
    angle it leaves out is the one the arm is already holding. Nulls stay
    inside the resolver, where they mean "unchanged", and never reach the file.
    """
    state = load_state()
    absolute = {"action": command["action"], "direction": command["direction"]}
    for joint in ALL_JOINTS:
        value = command.get(joint)
        absolute[joint] = value if _in_range(value, *JOINT_RANGES[joint]) else state[joint]
    return absolute


def save_command(command: dict) -> None:
    # Writing the absolute form here means no caller can put a null on disk.
    _write_json(COMMAND_FILE, absolute_command(command))


# =====================================================================
# 3. THE LOCAL MODEL
# =====================================================================

def _load_model():
    """Load the fine-tuned Qwen3 once, in 8-bit, from disk only."""
    global MODEL, TOKENIZER
    with MODEL_LOCK:
        if MODEL is None:
            import torch  # noqa: F401  (imported for its side effects on load)
            from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig

            model = AutoModelForCausalLM.from_pretrained(
                str(MODEL_PATH),
                quantization_config=BitsAndBytesConfig(
                    load_in_8bit=True, bnb_8bit_compute_dtype=torch.bfloat16
                ),
                device_map="auto",
                local_files_only=True,
            )
            tokenizer = AutoTokenizer.from_pretrained(
                str(MODEL_PATH), local_files_only=True
            )
            if tokenizer.pad_token is None:
                tokenizer.pad_token = tokenizer.eos_token
            MODEL, TOKENIZER = model, tokenizer
    return MODEL, TOKENIZER


def ask_model(prompt: str) -> dict:
    """Hand the request to the local model and rebuild a full command from the
    two fields it returns.  Raises ValueError when the model is unusable."""
    import torch

    model, tokenizer = _load_model()
    prompt_text = tokenizer.apply_chat_template(
        [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": prompt},
        ],
        tokenize=False,
        add_generation_prompt=True,
        enable_thinking=False,
    )
    inputs = tokenizer(prompt_text, return_tensors="pt").to(model.device)
    with torch.no_grad():
        outputs = model.generate(
            **inputs,
            max_new_tokens=64,
            do_sample=False,
            pad_token_id=tokenizer.eos_token_id,
            eos_token_id=tokenizer.eos_token_id,
        )
    raw = tokenizer.decode(
        outputs[0][len(inputs.input_ids[0]):], skip_special_tokens=True
    ).strip()

    intent = parse_intent(raw)
    if intent is None:
        raise ValueError("the model did not return an action and a direction")
    action, direction = intent
    overrides = local_overrides(prompt)
    command = resolve_command(coerce_action(action, overrides), direction, overrides)
    if not is_valid_command(command):
        raise ValueError("the model returned an invalid robot command")
    return command


def ask_brain(prompt: str) -> dict:
    """The whole brain: parser first, model only when the parser abstains.

    The Command tab splits these two calls apart so the model can run off the
    UI thread; this is the one-shot form for scripts.
    """
    local = build_local_command(prompt)
    if local is not None:
        return local
    return ask_model(prompt)


# =====================================================================
# 4. GEOMETRY
# =====================================================================

@dataclass(frozen=True)
class RobotConfig:
    l0: float = L0
    l1: float = L1
    l2: float = L2
    l3: float = L3
    finger_count: int = FINGER_COUNT
    finger_length: float = FINGER_LENGTH
    finger_spread: float = FINGER_SPREAD_DEG
    finger_curl: float = FINGER_CURL_DEG

    @property
    def gripper_opening(self) -> float:
        return 1.0


@dataclass(frozen=True)
class JointAngles:
    """Relative joint angles in degrees; joint1 + joint2 + joint3 is the tool
    elevation down the arm. ``gripper_roll`` is the legacy name for joint4."""

    theta_base: float
    joint1: float
    joint2: float
    joint3: float
    joint4: float = 0.0

    @property
    def gripper_roll(self) -> float:
        """Backward-compatible alias for the wrist/tool roll joint."""
        return self.joint4


@dataclass(frozen=True)
class CartesianPoint:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class GripperPose:
    joint1: CartesianPoint
    joint2: CartesianPoint
    joint3: CartesianPoint
    gripper_mount: CartesianPoint
    spread: float
    fingertips: tuple[CartesianPoint, ...]
    # Each rigid finger's centreline from base to tip, so the claw can be drawn
    # as a curve; `fingertips` is the last point of each path.
    finger_paths: tuple[tuple[CartesianPoint, ...], ...] = ()
    gripper_roll: float = 0.0

    @property
    def joint4(self) -> float:
        """The wrist/tool roll angle, named as a joint."""
        return self.gripper_roll


CONFIG = RobotConfig()
HOME_ANGLES = JointAngles(DEFAULT_STATE["base"], DEFAULT_STATE["joint1"],
                          DEFAULT_STATE["joint2"], DEFAULT_STATE["joint3"],
                          DEFAULT_STATE["gripper_roll"])
POSITION_TOLERANCE_MM = 0.5
# Boundary slack on the geometric reach test, and the radius below which the
# law of cosines runs into a degenerate triangle.
REACH_EPSILON_MM = 1e-9
DEGENERATE_SPAN_MM = 1e-9


def _add(a: CartesianPoint, b: CartesianPoint) -> CartesianPoint:
    return CartesianPoint(a.x + b.x, a.y + b.y, a.z + b.z)


def _scale(v: CartesianPoint, k: float) -> CartesianPoint:
    return CartesianPoint(v.x * k, v.y * k, v.z * k)


def _cross(a: CartesianPoint, b: CartesianPoint) -> CartesianPoint:
    return CartesianPoint(
        a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x
    )


def _unit(v: CartesianPoint) -> CartesianPoint:
    length = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    if length == 0.0:
        raise ValueError("Cannot normalize a zero-length vector.")
    return _scale(v, 1.0 / length)


def check_joint_limits(angles: JointAngles, config: RobotConfig = CONFIG) -> bool:
    return all(
        _in_range(getattr(angles, field), *JOINT_RANGES[joint])
        for field, joint in ANGLE_JOINTS
    )


def _require_limits(angles: JointAngles, config: RobotConfig = CONFIG) -> None:
    if not check_joint_limits(angles, config):
        raise ValueError(
            "The solution leaves a joint outside its range: "
            f"base {angles.theta_base:.2f}, joint1 {angles.joint1:.2f}, "
            f"joint2 {angles.joint2:.2f}, joint3 {angles.joint3:.2f}, "
            f"joint4 {angles.joint4:.2f}."
        )


def gripper_spread(
    opening: float | None, config: RobotConfig = CONFIG
) -> float:
    """Radians a finger sits off the tool axis, in radians."""
    if opening is None:
        opening = config.gripper_opening
    if not 0.0 <= opening <= 1.0:
        raise ValueError("Gripper opening must be between 0 and 1.")
    return math.radians(config.finger_spread) * opening


def _finger_curl(config: RobotConfig) -> float:
    """Total inward curl of one rigid finger, in radians."""
    return math.radians(config.finger_curl)


def _finger_arc_radius(config: RobotConfig) -> float:
    """Radius of the circular arc a rigid finger follows."""
    alpha = _finger_curl(config)
    return config.finger_length / alpha if alpha else config.finger_length


def _finger_base_radius(config: RobotConfig) -> float:
    """Radius of the ring the finger bases sit on, so closed tips meet the axis."""
    return _finger_arc_radius(config) * (1.0 - math.cos(_finger_curl(config)))


def _grip_depth(config: RobotConfig) -> float:
    """On-axis depth of the grasp centre in mm.  A CONSTANT, so the gripper
    opening never reaches the arm's kinematics."""
    return _finger_arc_radius(config) * math.sin(_finger_curl(config))


def _finger_arc(
    mount: CartesianPoint,
    axis: CartesianPoint,
    radial: CartesianPoint,
    spread: float,
    config: RobotConfig,
    samples: int = FINGER_PATH_SAMPLES,
) -> tuple[CartesianPoint, ...]:
    """Centreline of ONE rigid curled finger, from base to tip.

    The finger is a circular arc of fixed curvature -- a talon.  It is one rigid
    piece: there is no joint anywhere along it, and the opening only swings the
    whole piece about its base.  With arc radius ``R``, total curl ``alpha`` and
    arc length ``L``, the tangent has swept ``theta(s) = spread - alpha*s/L`` by
    arc length ``s``, so integrating the tangent gives

        axial(s)  = R * (sin(spread) - sin(spread - alpha*s/L))
        radial(s) = R_base + R * (cos(spread - alpha*s/L) - cos(spread))

    Both expressions are continuous as ``alpha -> 0``, where they reduce to the
    straight finger ``s*cos(spread)`` / ``s*sin(spread)``; that limit is handled
    explicitly to avoid dividing by zero.
    """
    alpha = _finger_curl(config)
    length = config.finger_length
    arc = _finger_arc_radius(config)
    base_radius = _finger_base_radius(config)

    points: list[CartesianPoint] = []
    for index in range(samples + 1):
        s = length * index / samples
        if alpha <= 1e-12:
            axial, rad_off = s * math.cos(spread), s * math.sin(spread)
        else:
            theta = spread - alpha * s / length
            axial = arc * (math.sin(spread) - math.sin(theta))
            rad_off = base_radius + arc * (math.cos(theta) - math.cos(spread))
        points.append(
            _add(mount, _add(_scale(axis, axial), _scale(radial, rad_off)))
        )
    return tuple(points)


def _plane_pose(angles: JointAngles, config: RobotConfig) -> tuple[float, ...]:
    """Radial-Z plane solution: (r2, h2, r3, h3, rg, hg, a1, a2, a3)."""
    a1 = math.radians(angles.joint1)
    a2 = a1 + math.radians(angles.joint2)
    a3 = a2 + math.radians(angles.joint3)
    r2 = config.l1 * math.cos(a1)
    h2 = config.l0 + config.l1 * math.sin(a1)
    r3 = r2 + config.l2 * math.cos(a2)
    h3 = h2 + config.l2 * math.sin(a2)
    rg = r3 + config.l3 * math.cos(a3)
    hg = h3 + config.l3 * math.sin(a3)
    return r2, h2, r3, h3, rg, hg, a1, a2, a3


def _place(radial: float, height: float, yaw: float) -> CartesianPoint:
    return CartesianPoint(radial * math.cos(yaw), radial * math.sin(yaw), height)


def _tool_reach(opening: float, config: RobotConfig) -> float:
    """Distance from the wrist joint to the grip point along the tool axis."""
    gripper_spread(opening, config)
    return config.l3 + _grip_depth(config)


def forward_kinematics(
    angles: JointAngles, opening: float = 1.0, config: RobotConfig = CONFIG
) -> CartesianPoint:
    """World position of the grip point for a pose."""
    _require_limits(angles, config)
    yaw = math.radians(angles.theta_base)
    _, _, r3, h3, _, _, _, _, a3 = _plane_pose(angles, config)
    tool = _tool_reach(opening, config)
    return _place(r3 + tool * math.cos(a3), h3 + tool * math.sin(a3), yaw)


def gripper_pose(
    angles: JointAngles, opening: float = 1.0, config: RobotConfig = CONFIG
) -> GripperPose:
    """Every joint origin, the tool axis and the fingertips for a pose."""
    _require_limits(angles, config)
    yaw = math.radians(angles.theta_base)
    r2, h2, r3, h3, rg, hg, _, _, a3 = _plane_pose(angles, config)

    base = CartesianPoint(0.0, 0.0, 0.0)
    j1 = CartesianPoint(0.0, 0.0, config.l0)
    mount = _place(rg, hg, yaw)
    spread = gripper_spread(opening, config)

    axis = CartesianPoint(
        math.cos(a3) * math.cos(yaw), math.cos(a3) * math.sin(yaw), math.sin(a3)
    )
    helper = CartesianPoint(0.0, 0.0, 1.0) if abs(axis.z) < 0.999 else CartesianPoint(1.0, 0.0, 0.0)
    first = _unit(_cross(axis, helper))
    second = _cross(axis, first)
    roll = math.radians(angles.joint4)
    first, second = (
        _add(_scale(first, math.cos(roll)), _scale(second, math.sin(roll))),
        _add(_scale(second, math.cos(roll)), _scale(first, -math.sin(roll))),
    )
    tips: list[CartesianPoint] = []
    paths: list[tuple[CartesianPoint, ...]] = []
    for index in range(config.finger_count):
        azimuth = 2.0 * math.pi * index / config.finger_count
        radial_dir = _add(
            _scale(first, math.cos(azimuth)), _scale(second, math.sin(azimuth))
        )
        path = _finger_arc(mount, axis, radial_dir, spread, config)
        paths.append(path)
        tips.append(path[-1])

    return GripperPose(
        joint1=j1,
        joint2=_place(r2, h2, yaw),
        joint3=_place(r3, h3, yaw),
        gripper_mount=mount,
        spread=spread,
        fingertips=tuple(tips),
        finger_paths=tuple(paths),
        gripper_roll=angles.joint4,
    )


def is_reachable(
    x: float, y: float, z: float, joint3: float | str = "auto",
    opening: float = 1.0, config: RobotConfig = CONFIG,
) -> bool:
    """True when the grip point can be put on (x, y, z) inside the joint limits.

    Delegates to the solver, so a target that reports reachable really does have
    a pose behind it.
    """
    if not all(math.isfinite(value) for value in (x, y, z)):
        return False
    try:
        inverse_kinematics(x, y, z, opening=opening, config=config, joint3=joint3)
    except ValueError:
        return False
    return True


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def _wrap_deg(value: float) -> float:
    """Wrap an angle in degrees into (-180, 180]."""
    return (value + 180.0) % 360.0 - 180.0


def _forearm(joint3: float, opening: float, config: RobotConfig) -> tuple[float, float]:
    """The forearm plus the whole tool extension as one rigid vector.

    L3 and the on-axis finger grip depth ride at ``joint3`` to the forearm, so
    ``L2 + (L3 + L_g) * e^(i*joint3) == m * e^(i*delta)`` collapses the problem
    to a plain two-link arm of lengths (L1, m).
    """
    tool = _tool_reach(opening, config)
    real = config.l2 + tool * math.cos(joint3)
    imag = tool * math.sin(joint3)
    return math.hypot(real, imag), math.atan2(imag, real)


def _solve_plane(
    radial: float, height: float, forearm: float, config: RobotConfig, joint2_up: bool
) -> tuple[float, float]:
    """Law of cosines on the radial-Z plane; returns (joint1, joint2 + delta)."""
    length1 = config.l1
    span2 = radial * radial + height * height
    span = math.sqrt(span2)
    # d^2 = L1^2 + m^2 + 2*L1*m*cos(te), so cos(te) = (d^2 - L1^2 - m^2)/(2*L1*m)
    te_abs = math.acos(_clamp((span2 - length1**2 - forearm**2) / (2.0 * length1 * forearm), -1.0, 1.0))
    # psi is the bearing of the target, gamma the interior angle at joint 1.
    psi = math.atan2(height, radial)
    gamma = math.acos(_clamp((length1**2 + span2 - forearm**2) / (2.0 * length1 * span), -1.0, 1.0))
    if joint2_up:
        return psi + gamma, -te_abs
    return psi - gamma, te_abs


def _ik_once(
    radial: float, height: float, joint3: float, opening: float,
    config: RobotConfig, joint2_up: bool, gripper_roll: float = 0.0,
) -> JointAngles:
    """One exact solve for a given wrist angle.  Raises when unreachable."""
    forearm, delta = _forearm(joint3, opening, config)
    span = math.hypot(radial, height)
    if span < DEGENERATE_SPAN_MM:
        raise ValueError("That target sits on the joint1 axis, so the pose is undefined.")
    lo, hi = abs(config.l1 - forearm), config.l1 + forearm
    if not lo - REACH_EPSILON_MM <= span <= hi + REACH_EPSILON_MM:
        raise ValueError(
            f"That target is {span:.1f} mm from the shoulder joint, outside the "
            f"[{lo:.1f}, {hi:.1f}] mm the arm can span with this wrist angle."
        )
    joint1, effective = _solve_plane(radial, height, forearm, config, joint2_up)
    # An absolute direction is periodic, so an out-of-range raw joint1 angle
    # can still be a perfectly legal pose once it is wrapped back.
    return JointAngles(
        0.0,
        _wrap_deg(math.degrees(joint1)),
        _wrap_deg(math.degrees(effective - delta)),
        _wrap_deg(math.degrees(joint3)),
        gripper_roll,
    )


def _ik_stem_world(
    x: float, y: float, z: float, joint2_up: bool, opening: float,
    config: RobotConfig, stem_deg: float = STEM_DOWN_ANGLE_DEG,
    gripper_roll: float = 0.0,
) -> JointAngles:
    """Solve IK for a FIXED ABSOLUTE stem orientation, returning degrees.

    Used by the ``joint3="down"`` mode: the L3 stem plus the gripper grip depth
    points straight down in the WORLD frame, so the returned pose always obeys
    ``joint1 + joint2 + joint3 == stem_deg`` and the gripper approaches the
    target along a vertical line however the arm is folded.

    Because the stem is rigid, pinning its absolute direction makes its
    contribution to the grip point a known constant vector.  Peeling that vector
    off the target leaves a plain two-link ``(L1, L2)`` problem, which is why
    this stays closed-form.  ``joint3`` is recovered as a consequence of the
    constraint, not taken as an input.

    Uses the target's signed world-frame yaw directly; the base range covers
    the full ``[-180, 180]`` azimuth.
    """
    yaw = math.degrees(math.atan2(y, x))
    radial = math.hypot(x, y)
    height = z - config.l0

    stem = math.radians(stem_deg)
    tool = _tool_reach(opening, config)  # rigid L3 + grip depth

    # Peel the known stem vector off the target; what remains is what links 1
    # and 2 alone must reach.
    radial -= tool * math.cos(stem)
    height -= tool * math.sin(stem)

    span2 = radial * radial + height * height
    span = math.sqrt(span2)
    if span < DEGENERATE_SPAN_MM:
        raise ValueError("That target sits on the joint1 axis, so the pose is undefined.")
    lo, hi = abs(config.l1 - config.l2), config.l1 + config.l2
    if not lo - REACH_EPSILON_MM <= span <= hi + REACH_EPSILON_MM:
        raise ValueError(
            f"That target cannot be reached with a straight-down wrist: after "
            f"removing the stem offset the links must span {span:.1f} mm, outside "
            f"the [{lo:.1f}, {hi:.1f}] mm available. Pick another wrist angle, or "
            f"use auto."
        )

    # Plain two-link solve on the peeled-off point.  _solve_plane is given L2 as
    # the second link and returns (joint1, joint2 + delta); delta is zero here
    # because the stem is peeled off rather than folded in, so the second value
    # is already the physical joint2.
    joint1, joint2 = _solve_plane(radial, height, config.l2, config, joint2_up)
    joint3 = stem - joint1 - joint2
    # Preserve the positive 180-degree endpoint rather than wrapping it to -180.
    return JointAngles(
        yaw,
        _wrap_deg(math.degrees(joint1)),
        _wrap_deg(math.degrees(joint2)),
        _wrap_deg(math.degrees(joint3)),
        gripper_roll,
    )


def inverse_kinematics(
    x: float, y: float, z: float, joint2_up: bool = True,
    joint3: float | str = "auto", opening: float = 1.0,
    config: RobotConfig = CONFIG,
    gripper_roll: float = 0.0,
) -> JointAngles:
    """Solve for the joint angles that put the grip point on (x, y, z).

    ``joint3`` is a wrist angle in degrees RELATIVE to L2, or one of two strings:
    ``"auto"`` lets the solver pick one, preferring the natural rest angle;
    ``"down"`` instead pins the stem straight down in the world frame so the
    gripper approaches vertically.     ``joint2_up`` selects the elbow branch for numeric wrist angles. With
    ``joint3="auto"``, it is preferred first and the alternate branch is tried
    only if no limit-valid pose exists on the preferred branch. The fixed-
    orientation ``"down"`` mode also tries the alternate branch if necessary.
    Raises ``ValueError`` when no pose inside the joint limits exists.
    """
    gripper_spread(opening, config)
    for name, value in (("x", x), ("y", y), ("z", z)):
        if not math.isfinite(value):
            raise ValueError(f"Target coordinate {name} must be finite.")
    # The full signed base range lets the base point directly toward the target,
    # so the radial distance remains non-negative in every world quadrant.
    yaw = math.degrees(math.atan2(y, x))
    radial = math.hypot(x, y)
    height = z - config.l0

    if isinstance(joint3, str):
        mode = joint3.strip().lower()
        if mode == "down":
            # Absolute stem orientation: joint3 is solved, not chosen, so this
            # bypasses the auto wrist search. Prefer the requested branch but
            # retain the alternate-branch fallback for this constrained mode.
            last = ""
            for up in (joint2_up, not joint2_up):
                try:
                    pose = _ik_stem_world(
                        x, y, z, up, opening, config,
                        gripper_roll=gripper_roll,
                    )
                except ValueError as exc:
                    last = str(exc)
                    continue
                if check_joint_limits(pose, config):
                    return pose
            raise ValueError(
                f"No straight-down wrist pose reaches ({x:.1f}, {y:.1f}, {z:.1f}) mm "
                f"within the joint limits."
                + (f" Last reason: {last}" if last else "")
            )
        if mode != "auto":
            raise ValueError(
                f"Unknown joint3 mode {joint3!r}; expected degrees, 'auto', or 'down'."
            )
        wrists = _wrist_candidates()
    else:
        wrists = (math.radians(float(joint3)),)

    last = ""
    branches = (
        (joint2_up, not joint2_up)
        if isinstance(joint3, str)
        else (joint2_up,)
    )
    for branch in branches:
        for wrist in wrists:
            try:
                angles = _ik_once(
                    radial, height, wrist, opening, config, branch, gripper_roll
                )
            except ValueError as exc:
                last = str(exc)
                continue
            angles = JointAngles(
                yaw, angles.joint1, angles.joint2, angles.joint3, gripper_roll
            )
            if check_joint_limits(angles, config):
                return angles

    raise ValueError(
        f"No arm pose reaches ({x:.1f}, {y:.1f}, {z:.1f}) mm within the joint limits."
        + (f" Last reason: {last}" if last else "")
    )


def _wrist_candidates(step: float = 5.0) -> tuple[float, ...]:
    """Wrist angles to try, in radians, nearest the rest angle first."""
    lo, hi = JOINT_RANGES["joint3"]
    rest = WRIST_HOME
    seen: list[float] = []
    for offset in [0.0] + [k * step for k in range(1, int(max(rest - lo, hi - rest) / step) + 1)]:
        for value in (rest + offset, rest - offset):
            if lo <= value <= hi and value not in seen:
                seen.append(value)
    return tuple(math.radians(v) for v in seen)


def verify_fk_ik(samples: int = 20, opening: float = 1.0,
                 config: RobotConfig = CONFIG) -> str:
    """Round-trip random poses: FK -> IK -> FK and report the worst error."""
    if samples < 1:
        raise ValueError("samples must be at least 1.")
    gripper_spread(opening, config)
    rng = random.Random(7)
    worst = 0.0
    solved = 0
    for _ in range(samples):
        angles = JointAngles(
            rng.uniform(*JOINT_RANGES["base"]),
            rng.uniform(*JOINT_RANGES["joint1"]),
            rng.uniform(*JOINT_RANGES["joint2"]),
            rng.uniform(*JOINT_RANGES["joint3"]),
        )
        goal = forward_kinematics(angles, opening, config)
        _, delta = _forearm(
            math.radians(angles.joint3), opening, config
        )
        joint2_up = math.radians(angles.joint2) + delta <= 0.0
        try:
            back = inverse_kinematics(
                goal.x, goal.y, goal.z, joint2_up=joint2_up,
                opening=opening, config=config,
            )
        except ValueError:
            continue
        solved += 1
        again = forward_kinematics(back, opening, config)
        worst = max(worst, math.dist((goal.x, goal.y, goal.z), (again.x, again.y, again.z)))
    verdict = (
        "PASS"
        if solved == samples and worst <= POSITION_TOLERANCE_MM
        else "FAIL"
    )
    return (
        f"FK -> IK -> FK round trip over {samples} random poses\n"
        f"  solved            : {solved}/{samples}\n"
        f"  worst error       : {worst:.4f} mm (tolerance {POSITION_TOLERANCE_MM} mm)\n"
        f"  verdict           : {verdict}"
    )


def interpolate_joints(
    start: JointAngles, end: JointAngles, frames: int = VIZ_ANIM_FRAMES
) -> tuple[JointAngles, ...]:
    if frames < 2:
        raise ValueError("Animation must contain at least two frames.")
    base_delta = _wrap_deg(end.theta_base - start.theta_base)
    roll_delta = _wrap_deg(end.joint4 - start.joint4)
    out: list[JointAngles] = []
    for index in range(frames):
        t = index / (frames - 1)
        eased = t * t * (3.0 - 2.0 * t)  # smoothstep
        out.append(
            JointAngles(
                _wrap_deg(start.theta_base + base_delta * eased),
                start.joint1 + (end.joint1 - start.joint1) * eased,
                start.joint2 + (end.joint2 - start.joint2) * eased,
                start.joint3 + (end.joint3 - start.joint3) * eased,
                _wrap_deg(start.joint4 + roll_delta * eased),
            )
        )
    out[0] = start
    out[-1] = end
    return tuple(out)


def interpolate_opening(
    start: float, end: float, frames: int = VIZ_ANIM_FRAMES
) -> tuple[float, ...]:
    """Grip opening on the same smoothstep as interpolate_joints.

    The gripper has no motor, but the fingers are still actuated: this returns
    exactly ``frames`` openings so the fingers open/close in step with the arm
    instead of snapping.  ``[0] == start`` and ``[-1] == end``.
    """
    if frames < 2:
        raise ValueError("Animation must contain at least two frames.")
    values: list[float] = []
    for index in range(frames):
        t = index / (frames - 1)
        eased = t * t * (3.0 - 2.0 * t)  # smoothstep
        values.append(start + (end - start) * eased)
    values[0] = start
    values[-1] = end
    return tuple(values)


def animation_limits(
    frames: Iterable[JointAngles], opening: float = 1.0, target: Any = None,
    config: RobotConfig = CONFIG,
) -> tuple[tuple[float, float], ...]:
    frames = list(frames)
    if opening is None or isinstance(opening, (int, float)):
        openings = [opening] * len(frames)
    else:
        openings = list(opening)
        if len(openings) != len(frames):
            raise ValueError(
                f"opening sequence length {len(openings)} does not match "
                f"frame count {len(frames)}."
            )
    points: list[CartesianPoint] = []
    points.extend(
        CartesianPoint(BASE_RADIUS_MM * math.cos(angle), BASE_RADIUS_MM * math.sin(angle), 0.0)
        for angle in (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)
    )
    for angles, op in zip(frames, openings):
        pose = gripper_pose(angles, op, config)
        points.extend([pose.joint1, pose.joint2, pose.joint3, pose.gripper_mount])
        points.extend(pose.fingertips)
        for path in pose.finger_paths:
            points.extend(path)
        points.append(forward_kinematics(angles, op, config))
    if isinstance(target, CartesianPoint):
        points.append(target)
    reach = (config.l0 + config.l1 + config.l2 + config.l3) * 0.55
    points.extend([
        CartesianPoint(reach, 0.0, 0.0),
        CartesianPoint(0.0, reach, 0.0),
        CartesianPoint(0.0, 0.0, reach),
    ])
    axes = list(zip(*[(p.x, p.y, p.z) for p in points]))
    # Frame the arm tightly but always keep the base inside the view. One
    # shared span preserves equal aspect; centring each axis on its own bounds
    # avoids the large empty box that symmetric +/-span limits created.
    bounds = [(min(0.0, min(axis)), max(0.0, max(axis))) for axis in axes]
    span = max(max(hi - lo for lo, hi in bounds) * 1.08, 300.0)
    return tuple(
        ((lo + hi - span) / 2.0, (lo + hi + span) / 2.0)
        for lo, hi in bounds
    )


# =====================================================================
# 5. DRAWING
# =====================================================================

BASE_RADIUS_MM = 50.0


class RobotView:
    """Persistent artists for one Matplotlib 3D axes.

    Rebuilding a 3D axes every frame costs hundreds of
    milliseconds on TkAgg, which makes the animation stutter. Drawing the static
    geometry once and then only replacing the changing line data keeps each
    frame cheap. The default axis panes are also off because projecting them is
    expensive; the coloured X/Y/Z triad and a small reusable grid below give the
    same sense of position without that cost.
    """

    _GRID_LINES = 5

    def __init__(self, axis_view, config: RobotConfig = CONFIG) -> None:
        self.ax = axis_view
        self.cfg = config
        self._limits: tuple[tuple[float, float], ...] | None = None
        self._build()

    def _build(self) -> None:
        ax = self.ax
        ax.clear()

        reach = (self.cfg.l0 + self.cfg.l1 + self.cfg.l2 + self.cfg.l3) * 0.55
        for endpoint, color, label in (
            (CartesianPoint(reach, 0, 0), "tab:red", "X"),
            (CartesianPoint(0, reach, 0), "tab:green", "Y"),
            (CartesianPoint(0, 0, reach), "tab:blue", "Z"),
        ):
            ax.plot([0, endpoint.x], [0, endpoint.y], [0, endpoint.z],
                    color=color, linewidth=1.2)
            tip = _scale(endpoint, 1.08)
            ax.text(tip.x, tip.y, tip.z, label, color=color, fontsize=9)

        ax.plot([0], [0], [0], linestyle="None", marker="o",
                color="black", markersize=6)
        ax.text(0, 0, 0, "O", color="black", fontsize=8)

        ax.plot([0, 0], [0, 0], [0, self.cfg.l0],
                color="black", linewidth=5, label="fixed L0")
        radius, theta = np.meshgrid(
            np.linspace(0.0, BASE_RADIUS_MM, 12),
            np.linspace(0.0, 2.0 * math.pi, 64),
        )
        ax.plot_surface(
            radius * np.cos(theta),
            radius * np.sin(theta),
            np.zeros_like(radius),
            color="dimgray",
            alpha=0.45,
            linewidth=0,
            shade=False,
            label=f"base platform (R={BASE_RADIUS_MM:g} mm)",
        )

        def line(**kwargs):
            return ax.plot([], [], [], **kwargs)[0]

        self._grid = line(color="gray", linewidth=0.6, alpha=0.35)
        self._chain = line(color="black", linewidth=4)
        self._joints = line(linestyle="None", marker="o", markersize=7,
                            color="tab:orange")
        self._finger_paths = line(color="tab:cyan", linewidth=1.6)
        self._finger_tips = line(linestyle="None", marker="o", markersize=4,
                                 color="tab:cyan")
        self._palm = line(color="dimgray", linewidth=2)
        self._goal = line(linestyle="None", marker="o", markersize=5,
                          color="tab:red")
        self._target = line(color="tab:green", linestyle=":", linewidth=1)
        self._target.set_visible(False)
        # Keep the joint readout just INSIDE the axes: ax.set_title() draws
        # above the axes and gets clipped by the tight figure margin.
        self._title = ax.text2D(
            0.5, 0.98, "", transform=ax.transAxes,
            ha="center", va="top", fontsize=8,
        )

        ax.set_box_aspect((1.0, 1.0, 1.0))
        ax.set_axis_off()

    def clear(self) -> None:
        """Blank the canvas and rebuild the reusable artists."""
        self._limits = None
        self._build()

    @staticmethod
    def _arrays(points: Iterable[CartesianPoint]) -> tuple[np.ndarray, ...]:
        points = tuple(points)
        return (
            np.asarray([p.x for p in points], dtype=float),
            np.asarray([p.y for p in points], dtype=float),
            np.asarray([p.z for p in points], dtype=float),
        )

    @classmethod
    def _joined_arrays(cls, groups: Iterable[Iterable[CartesianPoint]]) -> tuple[np.ndarray, ...]:
        """Several polylines in one artist, separated by NaNs."""
        xs: list[float] = []
        ys: list[float] = []
        zs: list[float] = []
        for group in groups:
            points = tuple(group)
            if not points:
                continue
            x, y, z = cls._arrays(points)
            xs.extend(x.tolist())
            ys.extend(y.tolist())
            zs.extend(z.tolist())
            xs.append(np.nan)
            ys.append(np.nan)
            zs.append(np.nan)
        return (
            np.asarray(xs, dtype=float),
            np.asarray(ys, dtype=float),
            np.asarray(zs, dtype=float),
        )

    @staticmethod
    def _grid_ticks(lo: float, hi: float, count: int) -> tuple[float, ...]:
        """Evenly spaced ticks with zero added, so the grid crosses the base."""
        ticks = {float(value) for value in np.linspace(lo, hi, count)}
        if lo <= 0.0 <= hi:
            ticks.add(0.0)
        return tuple(sorted(ticks))

    def _draw_grid(self, limits: tuple[tuple[float, float], ...]) -> None:
        """Grid on the X=0, Y=0 and Z=0 planes, meeting at the robot base.

        Matplotlib's default 3D grid is drawn through the axis panes, which is
        the single most expensive part of a TkAgg redraw. These lines are just
        one reusable Line3D artist, rebuilt only when the view limits change.
        """
        (xlo, xhi), (ylo, yhi), (zlo, zhi) = limits
        xs = self._grid_ticks(xlo, xhi, self._GRID_LINES)
        ys = self._grid_ticks(ylo, yhi, self._GRID_LINES)
        zs = self._grid_ticks(zlo, zhi, self._GRID_LINES)
        # The coordinate planes themselves. Fall back to the near edge if a
        # view ever excludes the origin entirely.
        x0 = 0.0 if xlo <= 0.0 <= xhi else xlo
        y0 = 0.0 if ylo <= 0.0 <= yhi else ylo
        z0 = 0.0 if zlo <= 0.0 <= zhi else zlo
        segments: list[tuple[CartesianPoint, CartesianPoint]] = []
        for x in xs:
            # XY plane: y runs across, at the base height.
            segments.append((CartesianPoint(x, ylo, z0), CartesianPoint(x, yhi, z0)))
            # XZ plane: z runs up, on the Y=0 wall.
            segments.append((CartesianPoint(x, y0, zlo), CartesianPoint(x, y0, zhi)))
        for y in ys:
            segments.append((CartesianPoint(xlo, y, z0), CartesianPoint(xhi, y, z0)))
            segments.append((CartesianPoint(x0, y, zlo), CartesianPoint(x0, y, zhi)))
        for z in zs:
            segments.append((CartesianPoint(xlo, y0, z), CartesianPoint(xhi, y0, z)))
            segments.append((CartesianPoint(x0, ylo, z), CartesianPoint(x0, yhi, z)))
        self._grid.set_data_3d(*self._joined_arrays(segments))

    def draw(self, angles: JointAngles, opening: Optional[float] = None,
             target: Any = None,
             fixed_limits: tuple[tuple[float, float], ...] | None = None) -> None:
        """Repaint the arm without recreating any artists."""
        op = self.cfg.gripper_opening if opening is None else opening
        limits = fixed_limits
        if limits is None:
            limits = animation_limits((angles,), op, target, self.cfg)
        if limits != self._limits:
            self.ax.set_xlim(*limits[0])
            self.ax.set_ylim(*limits[1])
            self.ax.set_zlim(*limits[2])
            self._limits = limits
            self._draw_grid(limits)

        pose = gripper_pose(angles, op, self.cfg)
        goal = forward_kinematics(angles, op, self.cfg)
        self._chain.set_data_3d(*self._arrays(
            (pose.joint1, pose.joint2, pose.joint3, pose.gripper_mount)
        ))
        self._joints.set_data_3d(*self._arrays(
            (pose.joint1, pose.joint2, pose.joint3)
        ))

        paths = pose.finger_paths or tuple(
            (pose.gripper_mount, tip) for tip in pose.fingertips
        )
        self._finger_paths.set_data_3d(*self._joined_arrays(paths))
        self._finger_tips.set_data_3d(*self._arrays(pose.fingertips))

        bases = [path[0] for path in paths]
        palm_groups: list[list[CartesianPoint]] = [
            [pose.gripper_mount, base] for base in bases
        ]
        if len(bases) >= 2:
            palm_groups.append(bases + [bases[0]])
        self._palm.set_data_3d(*self._joined_arrays(palm_groups))

        self._goal.set_data_3d(*self._arrays((goal,)))
        if isinstance(target, CartesianPoint):
            self._target.set_data_3d(*self._arrays((pose.gripper_mount, target)))
            self._target.set_visible(True)
        else:
            self._target.set_visible(False)

        self._title.set_text(
            f"L0={self.cfg.l0:g} L1={self.cfg.l1:g} L2={self.cfg.l2:g} "
            f"L3={self.cfg.l3:g} Lf={self.cfg.finger_length:g} mm\n"
            f"base yaw {angles.theta_base:6.2f}   joint1 {angles.joint1:6.2f}   "
            f"joint2 {angles.joint2:6.2f}   joint3 {angles.joint3:6.2f}   "
            f"gripper_roll {angles.joint4:6.2f}   gripper {op * 100:5.1f}%"
        )


def draw_robot(
    axis_view, angles: JointAngles, opening: float = 1.0,
    config: RobotConfig = CONFIG, target: Any = None,
    fixed_limits: tuple[tuple[float, float], ...] | None = None,
) -> None:
    """Render the arm, the fixed L0 column and the coordinate triad."""
    RobotView(axis_view, config).draw(angles, opening, target, fixed_limits)


# =====================================================================
# 6. TKINTER WORKFLOW
# =====================================================================

class _Output(ttk.Frame):
    """A scrollable, read-only text area for a tab's results."""

    def __init__(self, parent, height: int = 11) -> None:
        super().__init__(parent)
        self.text = tk.Text(self, height=height, state="disabled", wrap="word",
                            font=("Consolas", 10))
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
    """A labelled number entry; only the opening field may be left blank."""

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
        # Replace the plain entry built by _Field with a bounded spinbox.
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
        return None
    value = _req_float(field, "opening")
    if not 0.0 <= value <= 1.0:
        raise ValueError(f"Invalid opening: must be in [0, 1]; got {value!r}.")
    return value


JOINT3_CHOICES = ("auto", "down") + tuple(str(v) for v in range(-60, 151, 5))


def state_to_angles(state: dict) -> JointAngles:
    """The four arm angles and gripper roll; opening is stored separately."""
    return JointAngles(
        float(state["base"]), float(state["joint1"]),
        float(state["joint2"]), float(state["joint3"]),
        float(state["gripper_roll"]),
    )


class DumEApp:
    """The five geometry tabs plus a Command tab wired to the brain."""

    def __init__(self, root: tk.Tk, config: RobotConfig = CONFIG) -> None:
        self.root = root
        self.cfg = config
        root.title(
            f"DUM-E - L0={config.l0:g} L1={config.l1:g} L2={config.l2:g} "
            f"L3={config.l3:g} mm, {config.finger_count}-finger gripper"
        )
        root.geometry("1120x760")
        root.minsize(860, 600)

        self.state = ensure_state_file()
        self.results: queue.Queue[tuple[int, dict | None, str | None]] = queue.Queue()
        self.model_requests: queue.Queue[tuple[int, str] | None] = queue.Queue(
            maxsize=MODEL_REQUEST_QUEUE_SIZE
        )
        self.request_token = 0
        self._closed = False
        self._poll_job: Optional[str] = None

        self._build_app(root)

        self._angles: Optional[JointAngles] = state_to_angles(self.state)
        self._opening: float = float(self.state["gripper"]) / 100.0
        self._anim_frames: tuple = ()
        self._anim_openings: tuple = ()
        self._anim_limits = None
        self._anim_idx = 0
        self._anim_job = None
        self._anim_target = None
        self._anim_opening: Optional[float] = None
        self._anim_stop_requested = False
        self._anim_start = 0.0
        self._anim_tick = 0

        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._draw(self._angles, opening=self._opening)
        self._poll_job = self.root.after(POLL_INTERVAL_MS, self._poll_results)
        self._model_worker_thread = threading.Thread(
            target=self._model_worker, name="dume-model-worker", daemon=True
        )
        self._model_worker_thread.start()

    # -- shared layout ------------------------------------------------
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
        top = UI_PAD_Y + JOINT_TOP_PAD if row == 0 else UI_PAD_Y
        field.grid(row=row, column=column, sticky="w",
                   padx=(0, UI_PAD_X), pady=(top, UI_PAD_Y))
        setattr(owner, name, field)

    @staticmethod
    def _grid_combo(parent, name: str, label: str, values, default: str,
                    row: int, column: int, owner) -> None:
        cell = ttk.Frame(parent)
        cell.grid(row=row, column=column, sticky="ew",
                  padx=(0, UI_PAD_X), pady=UI_PAD_Y)
        ttk.Label(cell, text=label, width=18, anchor="w",
                  font=UI_FONT).pack(side="left")
        combo = ttk.Combobox(cell, values=values, width=10, state="readonly",
                             font=UI_FONT)
        combo.set(default)
        combo.pack(side="left", fill="x", expand=True)
        setattr(owner, name, combo)

    def _joint_values(self, parent, prefix: str, base="45.0", j1="30.0",
                      j2="-20.0", j3="-60.0", roll="0.0", opening="1.0",
                      j3_name=None, j4_name=None, open_name=None,
                      roll_alias=None, compact=False):
        """Build the six editable values as one aligned ``3 x 2`` grid.

        ``compact`` is accepted for older callers; the panel is always compact.
        ``roll_alias`` keeps the old ``<prefix>_roll`` attribute working.
        """
        grid = ttk.Frame(parent)
        for column in range(3):
            grid.columnconfigure(column, weight=1, uniform="joint")

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
                else JOINT_LIMITS[key]
            )
            self._grid_joint_field(grid, name, label, default, limits,
                                   index // 3, index % 3, self)
        setattr(self, roll_alias, getattr(self, j4_name))
        return grid

    def _angles_row(self, parent, prefix: str, **kwargs) -> None:
        """Backward-compatible wrapper around :meth:`_joint_values`."""
        self._joint_values(parent, prefix, **kwargs).pack(fill="x")

    def _read_angles(self, prefix: str) -> tuple[JointAngles, Optional[float]]:
        angles = JointAngles(
            _req_float(getattr(self, f"{prefix}_base"), "theta_base"),
            _req_float(getattr(self, f"{prefix}_j1"), "joint1"),
            _req_float(getattr(self, f"{prefix}_j2"), "joint2"),
            _req_float(getattr(self, f"{prefix}_j3"), "joint3"),
            _req_float(getattr(self, f"{prefix}_j4"), "joint4"),
        )
        if not check_joint_limits(angles, self.cfg):
            out_of_range = []
            for field, joint in ANGLE_JOINTS:
                value = getattr(angles, field)
                lo, hi = JOINT_RANGES[joint]
                if not lo <= value <= hi:
                    out_of_range.append(
                        f"{joint} {value:g} is outside {lo:g}..{hi:g}"
                    )
            raise ValueError(
                "Joint angles outside configured limits: "
                + "; ".join(out_of_range)
                + "."
            )
        return angles, _opt_opening(getattr(self, f"{prefix}_open"))

    # -- the single surface: command / joint angles / xyz target ------
    def _build_app(self, parent) -> None:
        """One window, one canvas, three ways to ask for a pose.

        Everything the old notebook carried now lives in one stack: a mode
        choice, the inputs for that mode, the buttons for that mode, and a
        single 3D canvas shared by all three. Only one input frame and one
        button row are packed at a time, so the mode swap is the same
        pack_forget trick the visualization tab already used.
        """
        tab = ttk.Frame(parent, padding=6)
        tab.pack(fill="both", expand=True)

        style = ttk.Style()
        style.configure("TButton", font=UI_FONT)
        style.configure("TRadiobutton", font=UI_FONT)
        style.configure("TLabelframe.Label", font=UI_FONT)

        # Mode choices and the active button row share one aligned header strip.
        header = ttk.Frame(tab)
        header.pack(fill="x", pady=(0, UI_PAD_Y))
        self.viz_mode = tk.StringVar(value="command")
        radios = ttk.Frame(header)
        radios.pack(side="left")
        for label, value in (("Command", "command"),
                             ("Joint angles", "angles"),
                             ("XYZ target", "target")):
            ttk.Radiobutton(radios, text=label, variable=self.viz_mode,
                            value=value, command=self._toggle_mode).pack(
                                side="left", padx=(0, 10))
        ttk.Separator(header, orient="vertical").pack(
            side="left", fill="y", padx=(2, 8))
        button_host = ttk.Frame(header)
        button_host.pack(side="right")

        # -- input frames, one per mode
        self.cmd_frame = ttk.LabelFrame(tab, text="Tell Dum-E what to do",
                                        padding=(6, PANEL_TOP_PAD))
        self.cmd_text = tk.Text(self.cmd_frame, height=2, wrap="word",
                                font=("Segoe UI", 10))
        self.cmd_text.pack(side="left", fill="both", expand=True)
        self.cmd_text.bind("<Control-Return>", lambda _e: self.submit_command())

        self.viz_ang = ttk.LabelFrame(tab, text="Joint values",
                                      padding=(6, PANEL_TOP_PAD))
        self._angles_row(self.viz_ang, "v")

        self.viz_tgt = ttk.LabelFrame(tab, text="XYZ target grip point (world, mm)",
                                       padding=(6, PANEL_TOP_PAD))
        self.viz_tgt.columnconfigure(0, weight=1)
        self.viz_tgt.columnconfigure(1, weight=1)
        self._grid_field(self.viz_tgt, "v_x", "x (mm)", "300.0", 0, 0, self)
        self._grid_field(self.viz_tgt, "v_y", "y (mm)", "120.0", 0, 1, self)
        self._grid_field(self.viz_tgt, "v_z", "z (mm)", "150.0", 1, 0, self)
        self._grid_combo(self.viz_tgt, "v_tgt_j3", "Joint 3", JOINT3_CHOICES,
                         "auto", 1, 1, self)
        self._grid_field(self.viz_tgt, "v_open_t", "Opening 0..1", "1.0", 2, 0, self)
        self._grid_field(self.viz_tgt, "v_roll_t", "J4 wrist roll", "", 2, 1, self)
        self._grid_combo(self.viz_tgt, "v_mode", "Joint 2 pose", ("up", "down"),
                         "up", 3, 0, self)

        # -- button rows, one per mode
        self.cmd_buttons = ttk.Frame(button_host)
        ttk.Button(self.cmd_buttons, text="Send  (Ctrl+Enter)",
                   command=self.submit_command,
                   padding=(6, 2)).pack(side="left", padx=3)
        self.cmd_stop_btn = ttk.Button(self.cmd_buttons, text="Stop",
                                       command=self.stop_now, padding=(6, 2))
        self.cmd_stop_btn.pack(side="left", padx=3)

        self.viz_buttons = ttk.Frame(button_host)
        self.viz_plot_btn = ttk.Button(self.viz_buttons, text="Plot",
                                       command=self._do_viz_plot, padding=(6, 2))
        self.viz_plot_btn.pack(side="left", padx=3)
        ttk.Button(self.viz_buttons, text="Clear", command=self._viz_clear,
                   padding=(6, 2)).pack(side="left", padx=3)
        self.viz_anim_btn = ttk.Button(self.viz_buttons, text="Animate",
                                       command=self._anim_run, padding=(6, 2))
        self.viz_anim_btn.pack(side="left", padx=3)

        self.cmd_status = tk.StringVar(value="Ready")
        ttk.Label(tab, textvariable=self.cmd_status, anchor="w",
                  font=UI_FONT).pack(fill="x", pady=(2, 0))

        # -- the one canvas, plus the pose and activity side pane
        panes = ttk.Panedwindow(tab, orient="horizontal")
        self.viz_panes = panes
        panes.pack(fill="both", expand=True, pady=(UI_PAD_Y, 0))
        canvas_frame = ttk.Frame(panes)
        side = ttk.Frame(panes, padding=(UI_PAD_X, 0, 0, 0))
        panes.add(canvas_frame, weight=6)
        panes.add(side, weight=1)

        self.viz_fig = _new_figure((7.2, 4.6))
        self.viz_ax = self.viz_fig.add_subplot(111, projection="3d")
        # Let the 3-D view use more of the canvas while leaving tick labels.
        self.viz_fig.subplots_adjust(left=0.03, right=0.985,
                                     bottom=0.03, top=0.97)
        self.viz_canvas = _new_canvas(self.viz_fig, canvas_frame)
        _new_toolbar(self.viz_canvas, canvas_frame)
        self.viz_view = RobotView(self.viz_ax, self.cfg)

        pose_frame = ttk.LabelFrame(side, text="Remembered pose", padding=6)
        pose_frame.pack(fill="x")
        pose_grid = ttk.Frame(pose_frame)
        pose_grid.pack(fill="x")
        pose_grid.columnconfigure(0, weight=1)
        self.pose_labels: dict[str, ttk.Label] = {}
        for row, key in enumerate(POSE_KEYS):
            ttk.Label(pose_grid, text=POSE_LABELS[key], width=6,
                      anchor="w", font=UI_FONT).grid(
                          row=row, column=0, sticky="w", pady=1)
            label = ttk.Label(pose_grid, text="-", font=VALUE_FONT,
                              anchor="e")
            label.grid(row=row, column=1, sticky="e", pady=1)
            self.pose_labels[key] = label

        log_frame = ttk.LabelFrame(side, text="Activity", padding=6)
        log_frame.pack(fill="both", expand=True, pady=(UI_PAD_Y * 2, 0))
        # A narrow text widget keeps the side pane compact so the 3-D canvas
        # gets the space instead of an 80-column default Text request.
        self.cmd_log = tk.Text(log_frame, height=8, width=28,
                               state="disabled", wrap="word", font=UI_FONT)
        bar = ttk.Scrollbar(log_frame, command=self.cmd_log.yview)
        self.cmd_log.configure(yscrollcommand=bar.set)
        self.cmd_log.pack(side="left", fill="both", expand=True)
        bar.pack(side="right", fill="y")

        self._toggle_mode()
        self._refresh_pose_labels()

    def _toggle_mode(self) -> None:
        """Show only the input frame and buttons that belong to this mode."""
        mode = self.viz_mode.get()
        for frame in (self.cmd_frame, self.viz_ang, self.viz_tgt):
            frame.pack_forget()
        for buttons in (self.cmd_buttons, self.viz_buttons):
            buttons.pack_forget()
        if mode == "command":
            self.cmd_frame.pack(fill="x", pady=(UI_PAD_Y, 0),
                                before=self.viz_panes)
            self.cmd_buttons.pack(side="left")
        elif mode == "angles":
            self.viz_ang.pack(fill="x", pady=(UI_PAD_Y, 0),
                              before=self.viz_panes)
            self.viz_buttons.pack(side="left")
        else:
            self.viz_tgt.pack(fill="x", pady=(UI_PAD_Y, 0),
                              before=self.viz_panes)
            self.viz_buttons.pack(side="left")

    def _log(self, message: str) -> None:
        self.cmd_log.configure(state="normal")
        self.cmd_log.insert("end", f"{time.strftime('%H:%M:%S')}  {message}\n")
        self.cmd_log.see("end")
        self.cmd_log.configure(state="disabled")

    def _refresh_pose_labels(self) -> None:
        for key in POSE_KEYS:
            value = self.state["gripper_roll"] if key == "joint4" else self.state[key]
            unit = "%" if key == "gripper" else "°"
            self.pose_labels[key].configure(text=f"{value:g}{unit}")

    def _persist_visual_pose(self, angles: Optional[JointAngles] = None,
                             opening: Optional[float] = None) -> None:
        resolved_angles = self._angles if angles is None else angles
        if resolved_angles is None:
            return
        if opening is None:
            opening = (
                self._opening
                if angles is None and self._opening is not None
                else self.cfg.gripper_opening
            )
        pose = {
            "base": resolved_angles.theta_base,
            "joint1": resolved_angles.joint1,
            "joint2": resolved_angles.joint2,
            "joint3": resolved_angles.joint3,
            "gripper_roll": resolved_angles.joint4,
            "gripper": 100.0 * opening,
        }
        save_pose_state(pose)
        self.state = pose
        self._refresh_pose_labels()

    def submit_command(self) -> None:
        prompt = self.cmd_text.get("1.0", "end").strip()
        if not prompt:
            return
        self.cmd_text.delete("1.0", "end")
        self.request_token += 1
        token = self.request_token
        self.cmd_status.set("Thinking...")
        self._log(f"you: {prompt}")

        local = build_local_command(prompt)
        if local is not None:
            self._apply_command(local, "parser")
            return

        self._log("parser was not sure - asking the model")
        self._queue_model_request(token, prompt)

    def _queue_model_request(self, token: int, prompt: str) -> None:
        try:
            self.model_requests.put_nowait((token, prompt))
        except queue.Full:
            try:
                self.model_requests.get_nowait()
            except queue.Empty:
                pass
            self.model_requests.put_nowait((token, prompt))

    def _model_worker(self) -> None:
        while True:
            request = self.model_requests.get()
            if request is None:
                return
            token, prompt = request
            try:
                self.results.put((token, ask_model(prompt), None))
            except Exception as exc:  # surfaced in the UI, never crashes the window
                self.results.put((token, None, str(exc)))

    def _poll_results(self) -> None:
        try:
            while True:
                token, command, error = self.results.get_nowait()
                if token != self.request_token:
                    continue
                if error is not None:
                    self.cmd_status.set("Model error - arm unchanged")
                    self._log(f"model error: {error}")
                elif command is not None:
                    self._apply_command(command, "model")
        except queue.Empty:
            pass
        except Exception as exc:
            self._log(f"model result error: {exc}")
        finally:
            if not self._closed:
                try:
                    self._poll_job = self.root.after(POLL_INTERVAL_MS, self._poll_results)
                except tk.TclError:
                    pass

    def _apply_command(self, command: dict, source: str) -> None:
        """Persist the command, fold it into the pose, and animate there."""
        if not is_valid_command(command):
            messagebox.showerror("DUM-E", "That command failed angle validation.",
                                 parent=self.root)
            return
        save_command(command)
        # Log the same absolute pose that went to the file, so what the user
        # reads here is exactly what the arm was told to hold.
        self._log(f"{command['action']} ({source}): "
                  f"{json.dumps(absolute_command(command), separators=(',', ':'))}")

        if command["action"] == "help":
            self._log(HELP_TEXT)
            self.cmd_status.set("Ready")
            return
        if command["action"] == "status":
            self._log("pose: " + json.dumps(self.state, separators=(",", ":")))
            self.cmd_status.set("Status")
            return
        if command["action"] == "stop":
            self._anim_stop()
            self.cmd_status.set("Stopped")
            return

        save_state(command)
        self.state = load_state()
        self._refresh_pose_labels()
        goal = state_to_angles(self.state)
        target_opening = float(self.state["gripper"]) / 100.0
        start_opening = (self._opening if self._opening is not None
                         else self.cfg.gripper_opening)
        # Animating a pose the arm is already holding would just burn a second
        # of redraws, so only move when the pose OR the grip opening changes
        # (opening the fingers is itself a motion worth animating).
        pose_changed = self._angles is None or not _same_pose(self._angles, goal)
        opening_changed = abs(target_opening - start_opening) > 1e-6
        animate = pose_changed or opening_changed
        self._goto_viz_angles(goal, target_opening, animate=animate,
                              from_opening=start_opening)
        # Movement and home have no other exit, so they have to clear the
        # "Thinking..." that submit_command set. Without this the line stayed
        # stuck on Thinking even though the parser had answered instantly.
        self.cmd_status.set("Home" if command["action"] == "home" else "Moved")

    def stop_now(self) -> None:
        self.request_token += 1  # ignore any model answer still in flight
        self._anim_stop()
        self.cmd_status.set("Stopped")

    def _resolve_viz_pose(self):
        mode = self.viz_mode.get()
        if mode == "angles":
            angles, opening = self._read_angles("v")
            return angles, None, opening
        if mode == "command":
            # Plot and Animate are hidden in command mode, so this is only
            # reachable by a stray call. Fall back to the remembered pose
            # rather than silently reading the hidden XYZ fields.
            return self._angles or state_to_angles(self.state), None, self._opening
        x = _req_float(self.v_x, "x")
        y = _req_float(self.v_y, "y")
        z = _req_float(self.v_z, "z")
        raw_j3 = self.v_tgt_j3.get().strip()
        j3 = raw_j3 if raw_j3 in ("auto", "down") else float(raw_j3)
        opening = _opt_opening(self.v_open_t)
        roll_text = self.v_roll_t.get().strip()
        roll = (
            float(roll_text) if roll_text
            else (self._angles.joint4 if self._angles is not None else 0.0)
        )
        target = CartesianPoint(x, y, z)
        angles = inverse_kinematics(x, y, z, joint2_up=self.v_mode.get() != "down",
                                    config=self.cfg, opening=opening, joint3=j3,
                                    gripper_roll=roll)
        return angles, target, opening

    def _do_viz_plot(self) -> None:
        try:
            angles, target, opening = self._resolve_viz_pose()
        except ValueError as exc:
            messagebox.showerror("DUM-E", str(exc))
            return
        # Persist the solved/entered target before any redraw; plotting is not
        # a motion step and must not gate the state file.
        self._persist_visual_pose(angles, opening)
        self._draw(angles, opening=opening, target=target)

    def _viz_clear(self) -> None:
        self.viz_view.clear()
        self.viz_canvas.draw_idle()

    def _draw(self, angles: JointAngles, opening: Optional[float] = None, target=None,
              fixed_limits=None) -> None:
        self.viz_view.draw(angles, opening, target, fixed_limits)
        self.viz_canvas.draw_idle()
        self._angles = angles
        self._opening = self.cfg.gripper_opening if opening is None else opening

    # -- animation ----------------------------------------------------
    def _cancel_anim_job(self) -> None:
        """Drop a pending frame so a new move cannot leave two timers running."""
        if self._anim_job is not None:
            try:
                self.root.after_cancel(self._anim_job)
            except tk.TclError:
                pass
            self._anim_job = None

    def _start_animation(self, goal: JointAngles, opening: Optional[float],
                         target: Any, from_opening: Optional[float],
                         start: Optional[JointAngles] = None) -> None:
        """Prepare the frames and start a wall-clock paced animation.

        The timer targets one tick every ``VIZ_ANIM_FRAME_MS``. If a redraw takes
        longer, the next tick is due immediately and the step below jumps to
        the right frame; the move still takes the same time and never queues up
        stale frames.
        """
        self._cancel_anim_job()
        target_opening = self.cfg.gripper_opening if opening is None else opening
        if from_opening is None:
            from_opening = target_opening
        start_angles = start if start is not None else (self._angles or goal)
        self._anim_frames = interpolate_joints(start_angles, goal)
        self._anim_openings = interpolate_opening(
            from_opening, target_opening, len(self._anim_frames)
        )
        self._anim_limits = animation_limits(
            self._anim_frames, self._anim_openings, target, self.cfg
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
        if self._anim_job is not None:
            self._anim_stop()
            return
        try:
            goal, target, opening = self._resolve_viz_pose()
        except ValueError as exc:
            messagebox.showerror("DUM-E", str(exc))
            return
        # The target is the arm's commanded pose. Save it immediately, before
        # interpolation or the first animation frame begins.
        self._persist_visual_pose(goal, opening)
        start = self._angles or state_to_angles(self.state)
        start_opening = (self._opening if self._opening is not None
                         else self.cfg.gripper_opening)
        target_opening = self.cfg.gripper_opening if opening is None else opening
        if _same_pose(start, goal) and abs(target_opening - start_opening) <= 1e-6:
            self._draw(goal, opening, target)
            messagebox.showinfo(
                "DUM-E",
                "Start and target poses are identical, so there is nothing to\n"
                "animate. Change the inputs and try again.",
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
        self._draw(self._anim_frames[index], self._anim_openings[index],
                   self._anim_target, self._anim_limits)
        if index >= frame_count - 1:
            self._anim_finish()
            return

        # Aim at the next absolute tick, not "N ms after this draw finished".
        # A slow frame therefore costs a skipped frame, not a slower animation.
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
        canvas = getattr(self, "viz_canvas", None)
        if canvas is not None:
            canvas.draw_idle()
        self.viz_anim_btn.configure(text="Animate")
        self.viz_plot_btn.configure(state="normal")

    def _goto_viz_angles(self, angles: JointAngles, opening=None, target=None,
                         animate: bool = False, from_opening=None) -> None:
        """Move the arm to a pose without disturbing the chosen mode.

        A typed command calls this, and the user may still be part way through
        a sentence, so the mode radio and the joint-angle fields are left
        exactly as they are. The resulting pose is still reported numerically
        in the Remembered pose panel, which _apply_command refreshes.
        """
        if animate:
            self._start_animation(angles, opening, target, from_opening,
                                  start=self._angles)
        else:
            self._cancel_anim_job()
            self._draw(angles, opening, target)

    def close(self) -> None:
        """Shut down cleanly: drop in-flight work and stop every timer."""
        self.request_token += 1  # ignore any model answer still in flight
        self._closed = True
        for job in (self._poll_job, self._anim_job):
            if job is not None:
                try:
                    self.root.after_cancel(job)
                except tk.TclError:
                    pass
        self._poll_job = self._anim_job = None
        while True:
            try:
                self.model_requests.get_nowait()
            except queue.Empty:
                break
        self.model_requests.put_nowait(None)
        self.root.destroy()


def _same_pose(a: JointAngles, b: JointAngles, tol: float = 1e-3) -> bool:
    return all(abs(getattr(a, f) - getattr(b, f)) <= tol
               for f in ("theta_base", "joint1", "joint2", "joint3", "joint4"))


def _new_figure(size):
    from matplotlib.figure import Figure

    return Figure(figsize=size)


def _new_canvas(figure, master):
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

    canvas = FigureCanvasTkAgg(figure, master=master)
    canvas.get_tk_widget().pack(fill="both", expand=True)
    return canvas


def _new_toolbar(canvas, master) -> None:
    from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk

    NavigationToolbar2Tk(canvas, master).update()


def main(config: RobotConfig = CONFIG) -> None:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")

    import matplotlib

    matplotlib.use("TkAgg")

    state = ensure_state_file()
    print(f"Dum-E ready. Model: {MODEL_PATH.name}")
    print(f"Remembered pose: {json.dumps(state, separators=(',', ':'))}")
    print("Type 'help' in the Command tab for what Dum-E understands.")

    root = tk.Tk()
    DumEApp(root, config=config)
    root.mainloop()


if __name__ == "__main__":
    main()
