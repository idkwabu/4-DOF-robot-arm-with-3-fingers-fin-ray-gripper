"""
DUM-E - a natural-language robot arm: brain, kinematics and GUI in one file.

This merges three previously separate programs:

    mind/mind.py           the language brain (parser + local Qwen3 model)
    movements/robot_arm.py forward/inverse kinematics, reachability, drawing
    movements/robot_arm_gui.py  the five-stage Tkinter workflow

Everything now shares ONE angle contract, the one the trained model was fitted
against and that ``robot_state.json`` already stores:

    base     0 .. 180   yaw about +Z, 0 = +X, 90 = +Y, 180 = -X
    joint1   0 .. 180   link L1 in the radial-Z plane, from horizontal outward
    joint2 -150 .. 150   link L2 RELATIVE to L1
    joint3 -150 .. 150   link L3 RELATIVE to L2
    gripper   0 .. 100   0 = closed, 100 = open

The link chain is base -> fixed vertical L0 -> joint1 -> L1 -> joint2 -> L2
-> joint3 -> L3 -> gripper mount.  ``joint1 + joint2 + joint3`` is the absolute
tool elevation, so the three angles CUMULATE down the arm; the geometry below
converts to the per-link absolutes the renderer needs.

Run it with:

    python Dum-E.py

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
import re
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Any, Iterable, Optional

BASE_DIR = Path(__file__).resolve().parent
MIND_DIR = BASE_DIR / "mind"
MODEL_PATH = MIND_DIR / "dum-e-qwen3-merged"
# Shared with mind/mind.py and the previous GUI, so whichever program the
# hardware is watching, both brains write the same command and the same pose.
COMMAND_FILE = MIND_DIR / "robot_command.json"
STATE_FILE = MIND_DIR / "robot_state.json"
PROMPT_FILE = MIND_DIR / "system_prompt.txt"

# =====================================================================
# 1. THE ANGLE CONTRACT
# =====================================================================

DEFAULT_STATE = {"base": 90, "joint1": 90, "joint2": -60, "joint3": -60, "gripper": 50}
ALL_JOINTS = tuple(DEFAULT_STATE)
JOINT_RANGES = {
    "base": (0, 180),
    "joint1": (0, 180),
    "joint2": (-150, 150),
    "joint3": (-150, 150),
    "gripper": (0, 100),
}
VERTICAL_JOINTS = ("joint1", "joint2", "joint3")
# JointAngles field name -> the key it has in JOINT_RANGES.  Only the base yaw
# is spelled differently, because the dataclass inherited the old name.
ANGLE_JOINTS = (("theta_base", "base"), ("joint1", "joint1"),
                ("joint2", "joint2"), ("joint3", "joint3"))

DIRECTION_PRESETS = {
    "up": {"joint1": 120, "joint2": -60, "joint3": -30},
    "down": {"joint1": 60, "joint2": -90, "joint3": -60},
    # A plain turn swings 45 degrees off the middle; "hard" swings the full 90
    # to the mechanical limit. Diagonals turn exactly as far as the plain turn
    # they are named after, so "move up-left" is not a bigger turn than "left".
    "left": {"base": 135},
    "right": {"base": 45},
    "hard-left": {"base": 180},
    "hard-right": {"base": 0},
    "up-left": {"base": 135, "joint1": 120, "joint2": -60, "joint3": -30},
    "up-right": {"base": 45, "joint1": 120, "joint2": -60, "joint3": -30},
    "down-left": {"base": 135, "joint1": 60, "joint2": -90, "joint3": -60},
    "down-right": {"base": 45, "joint1": 60, "joint2": -90, "joint3": -60},
    # Centering the base on its own mechanical middle. The compound forms pair
    # that centered base with the up or down tilt, exactly as up-left pairs the
    # left base with the same tilt, so middle axes stay base-only and only the
    # compounds may move the arm.
    "middle": {"base": 90},
    "up-middle": {"base": 90, "joint1": 120, "joint2": -60, "joint3": -30},
    "down-middle": {"base": 90, "joint1": 60, "joint2": -90, "joint3": -60},
}
# Which joints each direction may move, read off the presets so the two tables
# can never drift apart.
DIRECTION_AXES = {name: frozenset(preset) for name, preset in DIRECTION_PRESETS.items()}
ALL_DIRECTION_VALUES = frozenset(DIRECTION_PRESETS) | {"default"}
MIDDLE_BASE = 90
# The directions whose target is the base's own middle. Kept as a set because
# three separate places need to ask "is this a middle direction": the conflict
# guard, the bare-dispatch check, and the action table.
MIDDLE_DIRECTIONS = frozenset({"middle", "up-middle", "down-middle"})

MOVE_ACTIONS = {"turn", "tilt", "move"}
CONTROL_ACTIONS = {"home", "stop", "status", "help"}
ALL_ACTIONS = MOVE_ACTIONS | CONTROL_ACTIONS
COMMAND_KEYS = frozenset({"action", "direction", *ALL_JOINTS})

# Link lengths in millimetres and the 3-finger end effector.
L0 = 50.0
L1 = 300.0
L2 = 250.0
L3 = 70.0
FINGER_COUNT = 3
FINGER_LENGTH = 50.0
FINGER_SPREAD_DEG = 30.0
# Where the tool axis sits relative to L2 when the wrist is at rest; used to
# prefer a natural-looking IK solution over a contorted one.
WRIST_HOME = -60.0
# Absolute stem orientation for the ``joint3="down"`` IK mode: the L3 stem plus
# the gripper grip depth points straight down in the WORLD frame, so
# ``joint1 + joint2 + joint3 == STEM_DOWN_ANGLE_DEG`` for any reachable pose.
# Distinct from WRIST_HOME, which is a wrist angle RELATIVE to L2.
STEM_DOWN_ANGLE_DEG = -90.0

VIZ_ANIM_FRAMES = 30
VIZ_ANIM_INTERVAL_MS = 30
# How often the Command tab drains finished model requests off the worker thread.
POLL_INTERVAL_MS = 60

HELP_TEXT = (
    "I understand: turn left/right, tilt up/down, move up-left/up-right/"
    "down-left/down-right, open or close the gripper, set a joint to a number, "
    "go home, stop, status.\n\n"
    "A plain turn swings 45 degrees off the middle. Add 'all the way', "
    "'as far as you can', 'fully', 'completely', 'hard' or 'max' for the full "
    "90: 'turn all the way left' swings to the stop, 'turn left' does not.\n\n"
    "Say 'middle', 'center', 'straight ahead' or 'halfway' and the base goes "
    "back to 90 on its own, leaving every other joint where it is."
)

# =====================================================================
# 2. LANGUAGE BRAIN - pattern tables
# =====================================================================

MOVE_VERB = re.compile(
    r"\b(?:move|shift|go|reach|raise|lower|lift|drop|push|swing|point|slide|carry|direct|lean|bring)\b",
    re.IGNORECASE,
)
TURN_VERB = re.compile(r"\b(?:turn|swivel|rotate|spin|pivot|face|steer)\b", re.IGNORECASE)
TILT_VERB = re.compile(r"\b(?:tilt|tip|dip)\b", re.IGNORECASE)
DIRECTION_RE = {
    "up": re.compile(r"\b(?:up|upward|above)\b", re.IGNORECASE),
    "down": re.compile(r"\b(?:down|downward|below)\b", re.IGNORECASE),
    "left": re.compile(r"\b(?:left|leftward)\b", re.IGNORECASE),
    "right": re.compile(r"\b(?:right|rightward)\b", re.IGNORECASE),
}
DIRECTION_MIDDLE = re.compile(
    r"\b(?:middle|center|centre|straight|halfway)\b|(?<!go )\bahead\b", re.IGNORECASE
)
# An intensifier counts as "hard" only when it modifies a horizontal turn.
# HARD_NEAR keeps the side in the capture group, so "turn hard right" can
# never be labelled from a stray "left" elsewhere in the sentence. The
# HARD_ANYWHERE pair covers "as far as you can to the left", where the filler
# words make adjacency impossible; both phrases are unambiguous in a command.
HARD_NEAR = re.compile(
    r"\b(?:fully|completely|entirely|hard|far|max(?:imum|imal)?(?:ly)?)\s+"
    r"(?:to\s+(?:the\s+)?|toward(?:s)?\s+(?:the\s+)?|the\s+)?"
    r"(left|right|leftward|rightward)\b",
    re.IGNORECASE,
)
HARD_ANYWHERE = re.compile(r"\b(?:all\s+the\s+way|as\s+far\s+as)\b", re.IGNORECASE)
# Words allowed to surround a hard direction when that direction is the entire
# instruction. Anything outside this set means the sentence is about something
# else, so it belongs to the model.
HARD_BARE_FILLER = frozenset({
    "a", "an", "the", "to", "toward", "towards", "as", "far", "all", "way",
    "you", "can", "could", "please", "and", "it", "its", "arm", "robot",
    "base", "hard", "fully", "completely", "entirely", "max", "maximum",
    "maximal", "left", "right", "leftward", "rightward", "turn", "move",
    "go", "swing", "swivel", "rotate", "spin", "just", "now", "then",
})
# Same idea for a middle word used on its own. "middle" names no turn verb, so
# without this it reaches the model, which was never trained on the bare form
# and falls back to help -- moving every joint instead of just the base. The
# middle words have to be listed here too, or the all() check below rejects the
# bare word itself. "go ahead" is deliberately not reachable: the lookbehind in
# DIRECTION_MIDDLE already keeps that phrase out.
MIDDLE_BARE_FILLER = frozenset({
    "middle", "center", "centre", "straight", "halfway", "ahead",
    "up", "down", "upward", "downward", "above", "below",
    "raise", "raised", "lift", "lower", "lowered", "tilt", "tip", "dip",
    "a", "an", "the", "to", "toward", "towards", "back", "please", "and",
    "it", "its", "arm", "robot", "base", "position", "pose",
    "turn", "go", "move", "swing", "swivel", "rotate", "spin", "point",
    "face", "steer", "pivot", "return", "just", "now", "then",
})
RAISE_VERB = re.compile(r"\b(?:raise|lift|elevate|ascend)\b", re.IGNORECASE)
LOWER_VERB = re.compile(r"\b(?:lower|drop|descend|sink)\b", re.IGNORECASE)
EXPLICIT_TARGET = re.compile(
    r"\b(joint\s*[123]|j1|j2|j3|base|gripper|fingers?|hand|first|second|third|1st|2nd|3rd)"
    r"\b[^.\n\d-]{0,20}(-?\d+)\b",
    re.IGNORECASE,
)
STOP_PHRASES = frozenset({
    "hold still", "stay still", "keep still", "freeze", "do not move",
    "don't move", "stop", "stop now", "stop moving", "emergency stop",
    "emergency-stop", "halt", "halt the arm", "stop everything",
})
STOP_NEGATED = re.compile(r"\b(?:don'?t|do not|never|no need to)\s+stop\b", re.IGNORECASE)
# Home, status and help are answered from these words alone, so the obvious
# ones never pay for loading the model. "stop" is not here on purpose: it has
# its own phrase table with negation handling. Anything unmatched still falls
# through to the model, so these rules only ever remove work, never add risk.
CONTROL_PATTERNS = (
    ("home", re.compile(
        r"\bhome\b(?!\s+(?:depot|brew|town|body|page|room|side|run|bound))"
        r"|\b(?:rest|resting|start|starting|default|initial|original|zero|neutral)\s+position\b"
        r"|\bback\s+to\s+(?:the\s+)?(?:start|beginning|home)\b"
        r"|\bstand\s?by\b"
        r"|\breset\b",
        re.IGNORECASE)),
    ("status", re.compile(
        r"\bstatus\b"
        r"|\bwhere\s+(?:is|are)\s+(?:the\s+arm|you)\b"
        r"|\bhow(?:'s| is)\s+the\s+arm\b"
        r"|\bare\s+you\s+(?:there|ok|okay|moving|alive|ready)\b"
        r"|\bis\s+the\s+arm\b"
        r"|\bcurrent\s+(?:pose|position|angles?)\b",
        re.IGNORECASE)),
    ("help", re.compile(
        r"\bhelp\b"
        r"|\bwhat\s+can\s+you\s+do\b"
        r"|\bwhat\s+do\s+you\s+(?:do|understand|support)\b"
        r"|\blist\s+your\s+commands\b",
        re.IGNORECASE)),
)
# "don't go home yet" asks for no motion, so it must not be answered with the
# home command. Mirrors how STOP_NEGATED protects the stop phrases.
CONTROL_NEGATED = re.compile(
    r"\b(?:don'?t|do not|never|no need to)\s+(?:\w+\s+){0,2}home\b", re.IGNORECASE
)
JOINT_ALIASES = {
    "fingers": "gripper", "finger": "gripper", "hand": "gripper",
    "first": "joint1", "1st": "joint1",
    "second": "joint2", "2nd": "joint2",
    "third": "joint3", "3rd": "joint3",
}
JOINT_WORDS = {
    "one": 1, "1st": 1, "first": 1,
    "two": 2, "2nd": 2, "second": 2,
    "three": 3, "3rd": 3, "third": 3,
}
UNITS = {
    "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
    "seven": 7, "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12,
    "thirteen": 13, "fourteen": 14, "fifteen": 15, "sixteen": 16, "seventeen": 17,
    "eighteen": 18, "nineteen": 19, "twenty": 20, "thirty": 30, "forty": 40,
    "fifty": 50, "sixty": 60, "seventy": 70, "eighty": 80, "ninety": 90,
}
NUMBER_WORDS = "|".join(UNITS)
WORD_NUMBER = re.compile(
    rf"\b(?:minus\s+)?(?:{NUMBER_WORDS})(?:\s+(?:hundred|and|{NUMBER_WORDS}))*\b",
    re.IGNORECASE,
)
GRIPPER_NOUN = re.compile(r"\b(?:gripper|fingers?|hand|fist|claw)\b", re.IGNORECASE)
GRIPPER_GESTURE = re.compile(
    r"\b(?:open|unclench|release|let\s+go|drop|close|closed|clench|squeeze|firmly|grip|grab|hold)\b",
    re.IGNORECASE,
)
GRIPPER_HALF = re.compile(r"\bhalf(?:way)?\b")
GRIPPER_AMOUNT = re.compile(
    r"\b(?:gripper|fingers?|hand)\b[^.\n\d-]{0,20}(-?\d+)\b|\b(\d+)\s*(?:percent|%)\b",
    re.IGNORECASE,
)
GRIPPER_VALUES = (
    (re.compile(r"\b(?:gently|softly)\b"), 30),
    (re.compile(r"\b(?:open|unclench|release|let\s+go|drop)\b"), 100),
    (re.compile(r"\b(?:close|closed|clench|squeeze|fist|firmly)\b"), 0),
    (re.compile(r"\b(?:grip|grab|hold)\b"), 15),
)

SYSTEM_PROMPT = ""
if PROMPT_FILE.exists():
    SYSTEM_PROMPT = PROMPT_FILE.read_text(encoding="utf-8").strip()

MODEL: Any = None
TOKENIZER: Any = None
MODEL_LOCK = threading.Lock()


# =====================================================================
# 3. LANGUAGE BRAIN - normalisation and parsing
# =====================================================================

def _normalize_joint_words(prompt: str) -> str:
    text = prompt.lower()
    for word, index in JOINT_WORDS.items():
        text = re.sub(rf"\b{word}\s+joint\b", f"joint{index}", text)
        text = re.sub(rf"\bjoint\s*{word}\b", f"joint{index}", text)
    return re.sub(r"\bj([123])\b", r"joint\1", text)


def _words_to_digits(text: str) -> str:
    def value_of(words: list[str]) -> int:
        negative = words[0] == "minus"
        if negative:
            words = words[1:]
        total = 0
        for word in words:
            if word == "hundred":
                total = (total or 1) * 100
            elif word != "and":
                total += UNITS[word]
        return -total if negative else total

    return WORD_NUMBER.sub(lambda m: str(value_of(m.group(0).lower().split())), text)


def _normalize_prompt(prompt: str) -> str:
    return _words_to_digits(_normalize_joint_words(prompt))


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _in_range(value: Any, lo: float, hi: float) -> bool:
    return _is_number(value) and lo <= value <= hi


def _number(value: float) -> float | int:
    """Keep whole angles as ints so the JSON Dum-E writes stays tidy."""
    return int(value) if float(value).is_integer() else float(value)


def parse_explicit_values(prompt: str) -> dict:
    normalized = _normalize_prompt(prompt)
    overrides: dict = {}
    for match in EXPLICIT_TARGET.finditer(normalized):
        name = match.group(1).strip().lower()
        target = JOINT_ALIASES.get(name, name)
        if target not in ALL_JOINTS:
            continue
        value = float(match.group(2))
        if _in_range(value, *JOINT_RANGES[target]):
            overrides[target] = _number(value)
    both = re.search(
        r"\b(?:both|all)\s+joints?\b[^.\n\d-]{0,15}(-?\d+)\b", normalized, re.IGNORECASE
    )
    if both:
        value = float(both.group(1))
        for joint in VERTICAL_JOINTS:
            if _in_range(value, *JOINT_RANGES[joint]):
                overrides.setdefault(joint, _number(value))
    return overrides


def user_specifies_gripper(prompt: str) -> bool:
    return bool(GRIPPER_NOUN.search(prompt) or GRIPPER_GESTURE.search(prompt))


def gripper_from_prompt(prompt: str) -> float | None:
    normalized = _normalize_prompt(prompt)
    amount = GRIPPER_AMOUNT.search(normalized)
    if amount:
        value = float(next(group for group in amount.groups() if group is not None))
        if _in_range(value, *JOINT_RANGES["gripper"]):
            return _number(value)
    # "halfway" is only a gripper value when the gripper is actually named; the
    # rest of the gestures mean the gripper on their own.
    if GRIPPER_NOUN.search(prompt) and GRIPPER_HALF.search(normalized):
        return 50
    for pattern, value in GRIPPER_VALUES:
        if pattern.search(normalized):
            return value
    return None


def local_overrides(prompt: str) -> dict:
    overrides = parse_explicit_values(prompt)
    if "gripper" not in overrides and user_specifies_gripper(prompt):
        value = gripper_from_prompt(prompt)
        if value is not None:
            overrides["gripper"] = value
    return overrides


def is_stop_request(prompt: str) -> bool:
    text = " ".join(prompt.lower().replace("!", " ").replace(".", " ").split())
    if STOP_NEGATED.search(text):
        return False
    return any(phrase in text for phrase in STOP_PHRASES)


def control_command(prompt: str) -> str | None:
    """The control action this prompt plainly asks for, or None to ask the model.

    Only the four control actions are ever returned, and only when a keyword
    says so outright. A negated request such as "don't go home yet" returns
    None on purpose, so it reaches the model instead of moving the arm.
    """
    if CONTROL_NEGATED.search(prompt):
        return None
    for action, pattern in CONTROL_PATTERNS:
        if pattern.search(prompt):
            return action
    return None


def _implied_vertical(prompt: str) -> str | None:
    if RAISE_VERB.search(prompt) and not LOWER_VERB.search(prompt):
        return "up"
    if LOWER_VERB.search(prompt) and not RAISE_VERB.search(prompt):
        return "down"
    return None


def detect_direction(prompt: str) -> str | None:
    seen = {name: bool(regex.search(prompt)) for name, regex in DIRECTION_RE.items()}
    horizontal = "left" if seen["left"] and not seen["right"] else (
        "right" if seen["right"] and not seen["left"] else None
    )
    # Middle is a third horizontal option, so the compound rule below pairs it
    # with up or down for free. It only counts when the middle word is the
    # instruction itself: "center of gravity" and "replace the middle gearbox"
    # name something other than a pose, and a middle word sitting beside a real
    # left or right is genuinely ambiguous, so in both cases the middle word is
    # ignored here and the prompt is left for the model.
    if (
        horizontal is None
        and DIRECTION_MIDDLE.search(prompt)
        and (_is_bare_middle(prompt) or _has_direction_verb(prompt))
    ):
        horizontal = "middle"
    vertical = "up" if seen["up"] and not seen["down"] else (
        "down" if seen["down"] and not seen["up"] else None
    )
    if vertical is None and horizontal is not None:
        # "lower the arm to the left" never says "down", but lowering it while
        # turning is a compound move, not a plain turn.
        vertical = _implied_vertical(prompt)
    if vertical and horizontal:
        return f"{vertical}-{horizontal}"
    if vertical:
        return vertical
    if horizontal == "middle":
        # Checked before the hardening below, because an intensifier must never
        # swing a centered base off its middle.
        return horizontal
    if horizontal:
        # Only a pure horizontal turn can be hardened: an intensifier on a
        # diagonal has already returned above, so the arm keeps turning the
        # same 45 degrees as the plain turn it is named after.
        near = HARD_NEAR.search(prompt)
        if near:
            return "hard-left" if near.group(1).startswith("left") else "hard-right"
        if HARD_ANYWHERE.search(prompt):
            return f"hard-{horizontal}"
        return horizontal
    return _implied_vertical(prompt)


def _has_direction_verb(prompt: str) -> bool:
    return bool(
        MOVE_VERB.search(prompt) or TURN_VERB.search(prompt) or TILT_VERB.search(prompt)
    )


def _is_bare_hard_turn(prompt: str) -> bool:
    """True when a hard turn is the whole instruction, with no turn verb.

    "all the way left" names no verb, so without this it would reach the model,
    which knows only the soft labels and would answer with a 45 degree turn.
    Content words beyond the direction phrase mean the sentence is about
    something else -- "as far as the left wheel goes, replace the belt" is
    maintenance, not a command -- so that still goes to the model.
    """
    if not (HARD_NEAR.search(prompt) or HARD_ANYWHERE.search(prompt)):
        return False
    return all(word in HARD_BARE_FILLER for word in re.findall(r"[a-z]+", prompt.lower()))


def _is_bare_middle(prompt: str) -> bool:
    """True when a middle word is the whole instruction, with no turn verb.

    "middle" and "center" name no verb, so without this they would reach the
    model, which has no bare-middle training row and answers "did not return an
    action and a direction", falling back to help and moving every joint.
    Content words beyond the middle word mean the sentence is about something
    else -- "center of gravity" is physics, "replace the middle gearbox" is
    maintenance -- so that still goes to the model.
    """
    if not DIRECTION_MIDDLE.search(prompt):
        return False
    return all(word in MIDDLE_BARE_FILLER for word in re.findall(r"[a-z]+", prompt.lower()))


def resolve_command(
    action: str, direction: str = "default", overrides: dict | None = None
) -> dict:
    command: dict = {"action": action, "direction": direction}
    for joint in ALL_JOINTS:
        command[joint] = None
    if action == "home":
        command.update(DEFAULT_STATE)
    elif direction in DIRECTION_PRESETS:
        command.update(DIRECTION_PRESETS[direction])
    elif action == "turn" and direction == "default":
        command["base"] = MIDDLE_BASE
    if overrides:
        command.update({j: overrides[j] for j in ALL_JOINTS if j in overrides})
    return command


def coerce_action(action: str, overrides: dict) -> str:
    """A turn that also names a vertical joint is a compound move."""
    if action != "turn" or not any(joint in overrides for joint in VERTICAL_JOINTS):
        return action
    return "move"


def _touches(command: dict, joints: Iterable[str]) -> bool:
    return any(command.get(joint) is not None for joint in joints)


def is_valid_command(command: Any) -> bool:
    if not isinstance(command, dict) or set(command) != COMMAND_KEYS:
        return False
    action = command["action"]
    direction = command["direction"]
    if action not in ALL_ACTIONS or direction not in ALL_DIRECTION_VALUES:
        return False
    for joint, limits in JOINT_RANGES.items():
        value = command[joint]
        if value is not None and not _in_range(value, *limits):
            return False
    if action in CONTROL_ACTIONS and direction != "default":
        return False
    if action == "turn" and direction not in {"left", "right", "hard-left", "hard-right", "middle", "default"}:
        return False
    if action == "tilt" and direction not in {"up", "down", "default"}:
        return False
    if direction in DIRECTION_AXES and _touches(
        command, set(ALL_JOINTS) - DIRECTION_AXES[direction] - {"gripper"}
    ):
        return False
    if action == "turn" and (command["base"] is None or _touches(command, VERTICAL_JOINTS)):
        return False
    if action == "tilt" and command["base"] is not None:
        return False
    if action in MOVE_ACTIONS and not _touches(command, ALL_JOINTS):
        return False
    return True


def build_local_command(prompt: str) -> dict | None:
    """The deterministic parser: a command, or None to defer to the model."""
    if is_stop_request(prompt):
        return resolve_command("stop")

    direction = detect_direction(prompt)
    overrides = local_overrides(prompt)
    vertical = any(joint in overrides for joint in VERTICAL_JOINTS)

    # A hard turn is an instruction in its own right: "all the way left" and
    # "hard left" name no turn verb, so without this they would fall through to
    # the model, which only knows the soft labels and would answer with a 45
    # degree turn.
    bare_hard = (
        direction is not None
        and direction.startswith("hard-")
        and not overrides
        and _is_bare_hard_turn(prompt)
    )

    # A middle word is an instruction in its own right in the same way: "middle",
    # "center" and "straight ahead" name a turn without naming a turn verb, so
    # without this they would be detected and then never dispatched.
    bare_middle = direction in MIDDLE_DIRECTIONS and not _has_direction_verb(prompt)

    if direction is not None and (_has_direction_verb(prompt) or bare_hard or bare_middle):
        # A middle word beside a real left or right is ambiguous -- "turn left to
        # the middle" names two different targets -- so the model decides. A
        # middle word that produced the direction itself is not a conflict.
        if DIRECTION_MIDDLE.search(prompt) and direction not in MIDDLE_DIRECTIONS:
            return None
        if direction in {"left", "right", "hard-left", "hard-right", "middle"}:
            action = "turn"
        elif direction in {"up", "down"}:
            action = "tilt"
        else:
            action = "move"
        return resolve_command(coerce_action(action, overrides), direction, overrides)

    if direction is None and overrides:
        action = "turn" if "base" in overrides and not vertical else "move"
        return resolve_command(action, "default", overrides)

    # The control words are tried last, so a prompt that also carries a real
    # direction or a number still moves: "help me turn left" is a turn, not a
    # help request. Reaching here means nothing moved, so a bare control word
    # is the whole intent and the model is not needed to hear it.
    control = control_command(prompt)
    if control is not None:
        return resolve_command(control)

    return None


def _iter_json_objects(text: str):
    decoder = json.JSONDecoder()
    index = 0
    text = text.strip()
    while index < len(text):
        while index < len(text) and text[index] != "{":
            index += 1
        if index >= len(text):
            return
        try:
            obj, end = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            index += 1
            continue
        yield obj
        index += end


def parse_intent(raw: str) -> tuple[str, str] | None:
    for candidate in _iter_json_objects(raw):
        if not isinstance(candidate, dict):
            continue
        action = candidate.get("action")
        direction = candidate.get("direction", "default")
        if action in ALL_ACTIONS and direction in ALL_DIRECTION_VALUES:
            return action, direction
    return None


# =====================================================================
# 4. REMEMBERED POSE AND THE COMMAND FILE
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
    _write_json(STATE_FILE, state)
    return state


def save_state(command: dict) -> None:
    """Fold a command into the remembered pose.  A null or unusable angle keeps
    the remembered one, so the pose on disk only ever moves to a value the arm
    can actually hold."""
    if command["action"] == "home":
        _write_json(STATE_FILE, DEFAULT_STATE.copy())
        return
    if command["action"] not in MOVE_ACTIONS:
        return
    state = load_state()
    for joint in ALL_JOINTS:
        value = command.get(joint)
        if _in_range(value, *JOINT_RANGES[joint]):
            state[joint] = value
    _write_json(STATE_FILE, state)


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
# 5. THE LOCAL MODEL
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
# 6. GEOMETRY
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

    @property
    def gripper_opening(self) -> float:
        return 1.0


@dataclass(frozen=True)
class JointAngles:
    """Relative joint angles in degrees; joint1 + joint2 + joint3 is the tool
    elevation down the arm."""

    theta_base: float
    joint1: float
    joint2: float
    joint3: float


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


CONFIG = RobotConfig()
HOME_ANGLES = JointAngles(DEFAULT_STATE["base"], DEFAULT_STATE["joint1"],
                          DEFAULT_STATE["joint2"], DEFAULT_STATE["joint3"])
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
            f"joint2 {angles.joint2:.2f}, joint3 {angles.joint3:.2f}."
        )


def gripper_spread(opening: float, config: RobotConfig = CONFIG) -> float:
    """Radians a finger sits off the tool axis, in radians."""
    if not 0.0 <= opening <= 1.0:
        raise ValueError("Gripper opening must be between 0 and 1.")
    return math.radians(config.finger_spread) * opening


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
    return config.l3 + config.finger_length * math.cos(gripper_spread(opening, config))


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
    tips: list[CartesianPoint] = []
    for index in range(config.finger_count):
        azimuth = 2.0 * math.pi * index / config.finger_count
        radial_dir = _add(
            _scale(first, math.cos(azimuth)), _scale(second, math.sin(azimuth))
        )
        direction = _add(
            _scale(axis, math.cos(spread)), _scale(radial_dir, math.sin(spread))
        )
        tips.append(_add(mount, _scale(direction, config.finger_length)))

    return GripperPose(
        joint1=j1,
        joint2=_place(r2, h2, yaw),
        joint3=_place(r3, h3, yaw),
        gripper_mount=mount,
        spread=spread,
        fingertips=tuple(tips),
    )


def _reach_bounds(opening: float = 1.0, config: RobotConfig = CONFIG) -> tuple[float, float]:
    """Reach bounds for the grip point with the wrist free to take any angle."""
    return _reach_bounds_with_joint3(0.0, opening, config)


def _reach_bounds_with_joint3(
    joint3: float, opening: float = 1.0, config: RobotConfig = CONFIG
) -> tuple[float, float]:
    """Reach bounds for the grip point measured from the joint1 pivot when the
    wrist is held at ``joint3``.

    L3 and the finger extension form one rigid piece that sits at ``joint3`` to
    the forearm, so forearm and wrist add up as a single vector before the
    triangle inequality with L1 is applied.
    """
    wrist = config.l3 + config.finger_length * math.cos(gripper_spread(opening, config))
    phi = math.radians(joint3)
    forearm = math.hypot(
        config.l2 + wrist * math.cos(phi), wrist * math.sin(phi)
    )
    return abs(config.l1 - forearm), config.l1 + forearm


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
    config: RobotConfig, joint2_up: bool,
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
    # An absolute direction is periodic, so a raw joint1 well outside -180..180
    # can still be a perfectly legal pose once it is wrapped back.
    return JointAngles(
        0.0,
        _wrap_deg(math.degrees(joint1)),
        _wrap_deg(math.degrees(effective - delta)),
        _wrap_deg(math.degrees(joint3)),
    )


def _ik_stem_world(
    x: float, y: float, z: float, joint2_up: bool, opening: float,
    config: RobotConfig, stem_deg: float = STEM_DOWN_ANGLE_DEG,
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

    Mirrors the ``y < 0`` folded-back convention used by :func:`inverse_kinematics`
    so targets behind the base stay reachable.
    """
    if y < 0.0:
        yaw = math.degrees(math.atan2(-y, -x))
        radial = -math.hypot(x, y)
    else:
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
    # `yaw` is deliberately NOT wrapped: this module's base range is 0..180, and
    # _wrap_deg(180) would return -180 and fall outside it.
    return JointAngles(
        yaw,
        _wrap_deg(math.degrees(joint1)),
        _wrap_deg(math.degrees(joint2)),
        _wrap_deg(math.degrees(joint3)),
    )


def inverse_kinematics(
    x: float, y: float, z: float, joint2_up: bool = True,
    joint3: float | str = "auto", opening: float = 1.0,
    config: RobotConfig = CONFIG,
) -> JointAngles:
    """Solve for the joint angles that put the grip point on (x, y, z).

    ``joint3`` is a wrist angle in degrees RELATIVE to L2, or one of two strings:
    ``"auto"`` lets the solver pick one, preferring the natural rest angle;
    ``"down"`` instead pins the stem straight down in the world frame so the
    gripper approaches vertically.  ``joint2_up`` picks the branch, and
    the other branch is tried too whenever the preferred one cannot be reached.
    Raises ``ValueError`` when no pose inside the joint limits exists.
    """
    # A point behind the base axis is not out of reach: the links can fold back
    # past the column, so it is reached with a negative radial distance and the
    # yaw reflected into 0..180.  Either way the solver sees a plain radius.
    if y < 0.0:
        yaw = math.degrees(math.atan2(-y, -x))
        radial = -math.hypot(x, y)
    else:
        yaw = math.degrees(math.atan2(y, x))
        radial = math.hypot(x, y)
    height = z - config.l0

    if isinstance(joint3, str):
        mode = joint3.strip().lower()
        if mode == "down":
            # Absolute stem orientation: joint3 is solved, not chosen, so this
            # bypasses the auto wrist search.  Both elbow branches are still
            # tried, and joint limits still apply, exactly as below.
            last = ""
            for up in (joint2_up, not joint2_up):
                try:
                    pose = _ik_stem_world(x, y, z, up, opening, config)
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
    for wrist in wrists:
        for up in (joint2_up, not joint2_up):
            try:
                angles = _ik_once(radial, height, wrist, opening, config, up)
            except ValueError as exc:
                last = str(exc)
                continue
            angles = JointAngles(yaw, angles.joint1, angles.joint2, angles.joint3)
            if check_joint_limits(angles, config):
                return angles
        if not isinstance(joint3, str):
            break

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
        try:
            back = inverse_kinematics(goal.x, goal.y, goal.z, opening=opening, config=config)
        except ValueError:
            continue
        solved += 1
        again = forward_kinematics(back, opening, config)
        worst = max(worst, math.dist((goal.x, goal.y, goal.z), (again.x, again.y, again.z)))
    verdict = "PASS" if worst <= POSITION_TOLERANCE_MM else "FAIL"
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
    out: list[JointAngles] = []
    for index in range(frames):
        t = index / (frames - 1)
        eased = t * t * (3.0 - 2.0 * t)  # smoothstep
        out.append(
            JointAngles(
                start.theta_base + (end.theta_base - start.theta_base) * eased,
                start.joint1 + (end.joint1 - start.joint1) * eased,
                start.joint2 + (end.joint2 - start.joint2) * eased,
                start.joint3 + (end.joint3 - start.joint3) * eased,
            )
        )
    out[0] = start
    out[-1] = end
    return tuple(out)


def animation_limits(
    frames: Iterable[JointAngles], opening: float = 1.0, target: Any = None,
    config: RobotConfig = CONFIG,
) -> tuple[tuple[float, float], ...]:
    points: list[CartesianPoint] = []
    for angles in frames:
        pose = gripper_pose(angles, opening, config)
        points.extend([pose.joint1, pose.joint2, pose.joint3, pose.gripper_mount])
        points.extend(pose.fingertips)
        points.append(forward_kinematics(angles, opening, config))
    if isinstance(target, CartesianPoint):
        points.append(target)
    reach = (config.l0 + config.l1 + config.l2 + config.l3) * 0.55
    points.extend([
        CartesianPoint(reach, 0.0, 0.0),
        CartesianPoint(0.0, reach, 0.0),
        CartesianPoint(0.0, 0.0, reach),
    ])
    axes = list(zip(*[(p.x, p.y, p.z) for p in points]))
    span = max(max(axis) - min(axis) for axis in axes)
    span = max(span * 0.58, 300.0)
    return tuple(
        ((max(axis) + min(axis)) / 2.0 - span, (max(axis) + min(axis)) / 2.0 + span)
        for axis in axes
    )


# =====================================================================
# 7. DRAWING
# =====================================================================

def draw_robot(
    axis_view, angles: JointAngles, opening: float = 1.0,
    config: RobotConfig = CONFIG, target: Any = None,
    fixed_limits: tuple[tuple[float, float], ...] | None = None,
) -> None:
    """Render the arm, the fixed L0 column and the coordinate triad."""
    pose = gripper_pose(angles, opening, config)
    goal = forward_kinematics(angles, opening, config)
    base = CartesianPoint(0.0, 0.0, 0.0)
    axis_view.clear()

    reach = (config.l0 + config.l1 + config.l2 + config.l3) * 0.55
    for endpoint, color, label in (
        (CartesianPoint(reach, 0, 0), "tab:red", "X"),
        (CartesianPoint(0, reach, 0), "tab:green", "Y"),
        (CartesianPoint(0, 0, reach), "tab:blue", "Z"),
    ):
        axis_view.plot([0, endpoint.x], [0, endpoint.y], [0, endpoint.z],
                       color=color, linewidth=1.2)
        tip = _scale(endpoint, 1.08)
        axis_view.text(tip.x, tip.y, tip.z, label, color=color, fontsize=9)

    axis_view.plot([base.x, pose.joint1.x], [base.y, pose.joint1.y], [base.z, pose.joint1.z],
                   color="black", linewidth=5, label="fixed L0")
    chain = (pose.joint1, pose.joint2, pose.joint3, pose.gripper_mount)
    axis_view.plot([p.x for p in chain], [p.y for p in chain], [p.z for p in chain],
                   color="black", linewidth=4, label="tilt links")
    axis_view.scatter(base.x, base.y, base.z, color="black", s=70, marker="o", label="base")
    axis_view.scatter(pose.joint1.x, pose.joint1.y, pose.joint1.z,
                      color="tab:purple", s=70, marker="s", label="joint 1")
    axis_view.scatter(pose.joint2.x, pose.joint2.y, pose.joint2.z,
                      color="tab:orange", s=70, marker="o", label="joint 2")
    axis_view.scatter(pose.joint3.x, pose.joint3.y, pose.joint3.z,
                      color="tab:blue", s=100, marker="*", label="joint 3")
    for tip in pose.fingertips:
        axis_view.plot([pose.gripper_mount.x, tip.x], [pose.gripper_mount.y, tip.y],
                       [pose.gripper_mount.z, tip.z], color="tab:cyan", linewidth=0.8)
        axis_view.scatter(tip.x, tip.y, tip.z, color="tab:cyan", s=15)
    axis_view.scatter(goal.x, goal.y, goal.z, color="tab:red", s=25)
    if isinstance(target, CartesianPoint):
        axis_view.scatter(target.x, target.y, target.z, color="tab:green", s=60,
                          marker="+", label="target")

    axis_view.set_xlabel("X (mm)")
    axis_view.set_ylabel("Y (mm)")
    axis_view.set_zlabel("Z (mm)")
    axis_view.set_title(
        f"base yaw {angles.theta_base:6.2f}   joint1 {angles.joint1:6.2f}   "
        f"joint2 {angles.joint2:6.2f}   joint3 {angles.joint3:6.2f}   "
        f"gripper {opening * 100:5.1f}%"
    )
    limits = fixed_limits or animation_limits((angles,), opening, target, config)
    axis_view.set_xlim(*limits[0])
    axis_view.set_ylim(*limits[1])
    axis_view.set_zlim(*limits[2])
    axis_view.set_box_aspect((1.0, 1.0, 1.0))
    axis_view.legend(loc="upper left", fontsize=8)
    axis_view.grid(True, alpha=0.25)


# =====================================================================
# 8. TKINTER WORKFLOW
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
        return None
    value = _req_float(field, "opening")
    if not 0.0 <= value <= 1.0:
        raise ValueError(f"Invalid opening: must be in [0, 1]; got {value!r}.")
    return value


JOINT3_CHOICES = ("auto", "down") + tuple(str(v) for v in range(-60, 151, 5))


def state_to_angles(state: dict) -> JointAngles:
    """The four arm angles in a remembered pose; the gripper rides separately."""
    return JointAngles(
        float(state["base"]), float(state["joint1"]),
        float(state["joint2"]), float(state["joint3"]),
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
        root.geometry("1180x820")
        root.minsize(920, 640)

        self.state = ensure_state_file()
        self.results: queue.Queue[tuple[int, dict | None, str | None]] = queue.Queue()
        self.request_token = 0
        self._closed = False
        self._poll_job: Optional[str] = None

        self._build_app(root)

        self._angles: Optional[JointAngles] = state_to_angles(self.state)
        self._opening: float = float(self.state["gripper"]) / 100.0
        self._anim_frames: tuple = ()
        self._anim_limits = None
        self._anim_idx = 0
        self._anim_job = None
        self._anim_target = None
        self._anim_opening: Optional[float] = None
        self._anim_stop_requested = False

        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._draw(self._angles, opening=self._opening)
        self._poll_job = self.root.after(POLL_INTERVAL_MS, self._poll_results)

    # -- shared layout ------------------------------------------------
    def _angles_row(self, parent, prefix: str, base="45.0", j1="30.0",
                    j2="-20.0", j3="-60.0", opening="1.0") -> None:
        setattr(self, f"{prefix}_base", _Field(parent, "Base yaw / spin (deg)", base))
        setattr(self, f"{prefix}_j1", _Field(parent, "Joint 1 / L1 tilt (deg)", j1))
        setattr(self, f"{prefix}_j2", _Field(parent, "Joint 2 / L2 tilt (deg)", j2))
        setattr(self, f"{prefix}_j3", _Field(parent, "Joint 3 / L3 tilt (deg)", j3))
        setattr(self, f"{prefix}_open",
                _Field(parent, "opening 0..1 (blank = default)", opening))

    def _read_angles(self, prefix: str) -> tuple[JointAngles, Optional[float]]:
        angles = JointAngles(
            _req_float(getattr(self, f"{prefix}_base"), "theta_base"),
            _req_float(getattr(self, f"{prefix}_j1"), "joint1"),
            _req_float(getattr(self, f"{prefix}_j2"), "joint2"),
            _req_float(getattr(self, f"{prefix}_j3"), "joint3"),
        )
        if not check_joint_limits(angles, self.cfg):
            lo, hi = JOINT_RANGES["base"]
            raise ValueError(
                f"base {angles.theta_base:g} is outside {lo:g}..{hi:g}; "
                f"joint1 {angles.joint1:g} is outside {lo:g}..{hi:g}; "
                f"joint2 {angles.joint2:g} is outside "
                f"{JOINT_RANGES['joint2'][0]:g}..{JOINT_RANGES['joint2'][1]:g}; "
                f"joint3 {angles.joint3:g} is outside "
                f"{JOINT_RANGES['joint3'][0]:g}..{JOINT_RANGES['joint3'][1]:g}."
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
        tab = ttk.Frame(parent, padding=8)
        tab.pack(fill="both", expand=True)

        self.viz_mode = tk.StringVar(value="command")
        radios = ttk.Frame(tab)
        radios.pack(fill="x")
        for label, value in (("Command", "command"),
                             ("From joint angles", "angles"),
                             ("From XYZ target", "target")):
            ttk.Radiobutton(radios, text=label, variable=self.viz_mode,
                            value=value, command=self._toggle_mode).pack(side="left", padx=4)

        # -- input frames, one per mode
        self.cmd_frame = ttk.LabelFrame(tab, text="Tell Dum-E what to do", padding=8)
        self.cmd_text = tk.Text(self.cmd_frame, height=3, wrap="word",
                                font=("Segoe UI", 11))
        self.cmd_text.pack(side="left", fill="both", expand=True)
        self.cmd_text.bind("<Control-Return>", lambda _e: self.submit_command())

        self.viz_ang = ttk.LabelFrame(tab, text="Joint angles", padding=8)
        self._angles_row(self.viz_ang, "v")

        self.viz_tgt = ttk.LabelFrame(tab, text="XYZ target grip point (world, mm)",
                                       padding=8)
        self.v_x = _Field(self.viz_tgt, "x (mm)", "300.0")
        self.v_y = _Field(self.viz_tgt, "y (mm)", "120.0")
        self.v_z = _Field(self.viz_tgt, "z (mm)", "150.0")
        row = ttk.Frame(self.viz_tgt)
        ttk.Label(row, text="joint3 / L3 tilt (deg)", width=30, anchor="w").pack(side="left")
        self.v_tgt_j3 = ttk.Combobox(row, values=JOINT3_CHOICES, width=10, state="readonly")
        self.v_tgt_j3.set("auto")
        self.v_tgt_j3.pack(side="left", fill="x", expand=True)
        row.pack(fill="x", pady=3)
        self.v_open_t = _Field(self.viz_tgt, "opening 0..1 (blank = default)", "1.0")
        row = ttk.Frame(self.viz_tgt)
        ttk.Label(row, text="joint2 pose", width=30, anchor="w").pack(side="left")
        self.v_mode = ttk.Combobox(row, values=("up", "down"), width=10, state="readonly")
        self.v_mode.set("up")
        self.v_mode.pack(side="left", fill="x", expand=True)
        row.pack(fill="x", pady=3)

        # -- button rows, one per mode
        self.cmd_buttons = ttk.Frame(tab)
        ttk.Button(self.cmd_buttons, text="Send  (Ctrl+Enter)",
                   command=self.submit_command).pack(side="left", padx=3)
        self.cmd_stop_btn = ttk.Button(self.cmd_buttons, text="Stop",
                                       command=self.stop_now)
        self.cmd_stop_btn.pack(side="left", padx=3)

        self.viz_buttons = ttk.Frame(tab)
        self.viz_plot_btn = ttk.Button(self.viz_buttons, text="Plot robot",
                                       command=self._do_viz_plot)
        self.viz_plot_btn.pack(side="left", padx=3)
        ttk.Button(self.viz_buttons, text="Clear", command=self._viz_clear).pack(side="left", padx=3)
        self.viz_anim_btn = ttk.Button(self.viz_buttons, text="Animate",
                                       command=self._anim_run)
        self.viz_anim_btn.pack(side="left", padx=3)

        self.cmd_status = tk.StringVar(value="Ready")
        ttk.Label(tab, textvariable=self.cmd_status).pack(anchor="w", pady=(4, 0))

        # -- the one canvas, plus the pose and activity side pane
        panes = ttk.Panedwindow(tab, orient="horizontal")
        panes.pack(fill="both", expand=True, pady=(6, 0))
        canvas_frame = ttk.Frame(panes)
        side = ttk.Frame(panes, padding=(10, 0, 0, 0))
        panes.add(canvas_frame, weight=4)
        panes.add(side, weight=1)

        self.viz_fig = _new_figure((7.2, 4.6))
        self.viz_ax = self.viz_fig.add_subplot(111, projection="3d")
        self.viz_canvas = _new_canvas(self.viz_fig, canvas_frame)
        _new_toolbar(self.viz_canvas, canvas_frame)

        pose_frame = ttk.LabelFrame(side, text="Remembered pose", padding=10)
        pose_frame.pack(fill="x")
        self.pose_labels: dict[str, ttk.Label] = {}
        for key in ALL_JOINTS:
            row = ttk.Frame(pose_frame)
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=key, width=10).pack(side="left")
            label = ttk.Label(row, text="-", font=("Consolas", 10, "bold"))
            label.pack(side="right")
            self.pose_labels[key] = label

        log_frame = ttk.LabelFrame(side, text="Activity", padding=8)
        log_frame.pack(fill="both", expand=True, pady=(10, 0))
        self.cmd_log = tk.Text(log_frame, height=10, state="disabled", wrap="word",
                               font=("Segoe UI", 9))
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
            self.cmd_frame.pack(fill="x", pady=(6, 0))
            self.cmd_buttons.pack(fill="x", pady=4)
        elif mode == "angles":
            self.viz_ang.pack(fill="x", pady=(6, 0))
            self.viz_buttons.pack(fill="x", pady=4)
        else:
            self.viz_tgt.pack(fill="x", pady=(6, 0))
            self.viz_buttons.pack(fill="x", pady=4)

    def _log(self, message: str) -> None:
        self.cmd_log.configure(state="normal")
        self.cmd_log.insert("end", f"{time.strftime('%H:%M:%S')}  {message}\n")
        self.cmd_log.see("end")
        self.cmd_log.configure(state="disabled")

    def _refresh_pose_labels(self) -> None:
        for key in ALL_JOINTS:
            suffix = "%" if key == "gripper" else " deg"
            self.pose_labels[key].configure(text=f"{self.state[key]:g}{suffix}")

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
        threading.Thread(target=self._request_model, args=(token, prompt),
                         daemon=True).start()

    def _request_model(self, token: int, prompt: str) -> None:
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
        self._opening = float(self.state["gripper"]) / 100.0
        # Animating a pose the arm is already holding would just burn a second
        # of redraws, so only move when something actually changes.
        animate = not (self._angles and _same_pose(self._angles, goal))
        self._goto_viz_angles(goal, self._opening, animate=animate)
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
        target = CartesianPoint(x, y, z)
        angles = inverse_kinematics(x, y, z, joint2_up=self.v_mode.get() != "down",
                                    config=self.cfg, opening=opening, joint3=j3)
        return angles, target, opening

    def _do_viz_plot(self) -> None:
        try:
            angles, target, opening = self._resolve_viz_pose()
        except ValueError as exc:
            messagebox.showerror("DUM-E", str(exc))
            return
        self._draw(angles, opening=opening, target=target)

    def _viz_clear(self) -> None:
        self.viz_ax.clear()
        self.viz_canvas.draw_idle()

    def _draw(self, angles: JointAngles, opening: Optional[float] = None, target=None,
              fixed_limits=None) -> None:
        draw_robot(self.viz_ax, angles, self.cfg.gripper_opening if opening is None
                   else opening, self.cfg, target, fixed_limits)
        self.viz_canvas.draw_idle()
        self._angles = angles
        self._opening = self.cfg.gripper_opening if opening is None else opening

    # -- animation ----------------------------------------------------
    def _anim_run(self) -> None:
        if self._anim_job is not None:
            self._anim_stop()
            return
        try:
            goal, target, opening = self._resolve_viz_pose()
        except ValueError as exc:
            messagebox.showerror("DUM-E", str(exc))
            return
        start = self._angles or state_to_angles(self.state)
        if _same_pose(start, goal):
            self._draw(goal, opening, target)
            messagebox.showinfo(
                "DUM-E",
                "Start and target poses are identical, so there is nothing to\n"
                "animate. Change the inputs and try again.",
            )
            return
        self._anim_frames = interpolate_joints(start, goal)
        self._anim_limits = animation_limits(self._anim_frames, opening, target, self.cfg)
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
        self._draw(angles, self._anim_opening, self._anim_target, self._anim_limits)
        if self._anim_idx >= len(self._anim_frames):
            self._anim_finish()
        elif not self._anim_stop_requested:
            self._anim_job = self.root.after(VIZ_ANIM_INTERVAL_MS, self._anim_step)

    def _anim_stop(self) -> None:
        self._anim_stop_requested = True
        if self._anim_job is not None:
            try:
                self.root.after_cancel(self._anim_job)
            except tk.TclError:
                pass
            self._anim_job = None
        self._anim_finish()

    def _anim_finish(self) -> None:
        self._anim_job = None
        self.viz_anim_btn.configure(text="Animate")
        self.viz_plot_btn.configure(state="normal")

    def _goto_viz_angles(self, angles: JointAngles, opening=None, target=None,
                         animate: bool = False) -> None:
        """Move the arm to a pose without disturbing the chosen mode.

        A typed command calls this, and the user may still be part way through
        a sentence, so the mode radio and the joint-angle fields are left
        exactly as they are. The resulting pose is still reported numerically
        in the Remembered pose panel, which _apply_command refreshes.
        """
        if animate:
            self._anim_frames = interpolate_joints(self._angles or angles, angles)
            self._anim_limits = animation_limits(self._anim_frames, opening, None, self.cfg)
            self._anim_idx = 0
            self._anim_target = None
            self._anim_opening = opening
            self._anim_stop_requested = False
            self.viz_anim_btn.configure(text="Stop")
            self.viz_plot_btn.configure(state="disabled")
            self._anim_step()
        else:
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
        self.root.destroy()


def _same_pose(a: JointAngles, b: JointAngles, tol: float = 1e-3) -> bool:
    return all(abs(getattr(a, f) - getattr(b, f)) <= tol
               for f in ("theta_base", "joint1", "joint2", "joint3"))


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


def main() -> None:
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
    DumEApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
