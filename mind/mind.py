import json
import re
import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
MODEL_PATH = BASE_DIR / "dum-e-qwen3-merged"
COMMAND_FILE = BASE_DIR / "robot_command.json"
STATE_FILE = BASE_DIR / "robot_state.json"
PROMPT_FILE = BASE_DIR / "system_prompt.txt"

DEFAULT_STATE = {"base": 90, "joint1": 90, "joint2": -60, "joint3": -60, "gripper": 50}
ALL_JOINTS = tuple(DEFAULT_STATE)
JOINT_RANGES = {
    "base": (0, 180),
    "joint1": (0, 180),
    "joint2": (-150, 150),
    "joint3": (-150, 150),
    "gripper": (0, 100),
}

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
VERTICAL_JOINTS = ("joint1", "joint2", "joint3")
# Which joints each direction is allowed to move, read straight off the presets
# so the two tables can never drift apart.
DIRECTION_AXES = {name: frozenset(preset) for name, preset in DIRECTION_PRESETS.items()}
ALL_DIRECTION_VALUES = frozenset(DIRECTION_PRESETS) | {"default"}
# The directions whose target is the base's own middle. Kept as a set because
# three separate places need to ask "is this a middle direction": the conflict
# guard, the bare-dispatch check, and the action table.
MIDDLE_DIRECTIONS = frozenset({"middle", "up-middle", "down-middle"})
MIDDLE_BASE = 90

MOVE_ACTIONS = {"turn", "tilt", "move"}
CONTROL_ACTIONS = {"home", "stop", "status", "help"}
ALL_ACTIONS = MOVE_ACTIONS | CONTROL_ACTIONS
COMMAND_KEYS = frozenset({"action", "direction", *ALL_JOINTS})

MOVE_VERB = re.compile(r"\b(?:move|shift|go|reach|raise|lower|lift|drop|push|swing|point|slide|carry|direct|lean|bring)\b", re.IGNORECASE)
TURN_VERB = re.compile(r"\b(?:turn|swivel|rotate|spin|pivot|face|steer)\b", re.IGNORECASE)
TILT_VERB = re.compile(r"\b(?:tilt|tip|dip)\b", re.IGNORECASE)
DIRECTION_RE = {
    "up": re.compile(r"\b(?:up|upward|above)\b", re.IGNORECASE),
    "down": re.compile(r"\b(?:down|downward|below)\b", re.IGNORECASE),
    "left": re.compile(r"\b(?:left|leftward)\b", re.IGNORECASE),
    "right": re.compile(r"\b(?:right|rightward)\b", re.IGNORECASE),
}
DIRECTION_MIDDLE = re.compile(
    r"\b(?:middle|center|centre|straight|halfway)\b|(?<!go )\bahead\b",
    re.IGNORECASE,
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
EXPLICIT_TARGET = re.compile(r"\b(joint\s*[123]|j1|j2|j3|base|gripper|fingers?|hand|first|second|third|1st|2nd|3rd)\b[^.\n\d-]{0,20}(-?\d+)\b", re.IGNORECASE)
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
    "fingers": "gripper",
    "finger": "gripper",
    "hand": "gripper",
    "first": "joint1",
    "1st": "joint1",
    "second": "joint2",
    "2nd": "joint2",
    "third": "joint3",
    "3rd": "joint3",
}

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


JOINT_WORDS = {
    "one": 1, "1st": 1, "first": 1,
    "two": 2, "2nd": 2, "second": 2,
    "three": 3, "3rd": 3, "third": 3,
}


def _normalize_joint_words(prompt: str) -> str:
    text = prompt.lower()
    for word, index in JOINT_WORDS.items():
        text = re.sub(rf"\b{word}\s+joint\b", f"joint{index}", text)
        text = re.sub(rf"\bjoint\s*{word}\b", f"joint{index}", text)
    return re.sub(r"\bj([123])\b", r"joint\1", text)


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

    def repl(match: re.Match) -> str:
        return str(value_of(match.group(0).lower().split()))

    return WORD_NUMBER.sub(repl, text)


def _normalize_prompt(prompt: str) -> str:
    return _words_to_digits(_normalize_joint_words(prompt))


SYSTEM_PROMPT = PROMPT_FILE.read_text(encoding="utf-8").strip()

MODEL = None
TOKENIZER = None


def _load_model():
    global MODEL, TOKENIZER
    if MODEL is not None:
        return MODEL, TOKENIZER
    import torch
    from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig

    quantization = BitsAndBytesConfig(
        load_in_8bit=True,
        bnb_8bit_compute_dtype=torch.bfloat16,
    )
    model = AutoModelForCausalLM.from_pretrained(
        str(MODEL_PATH),
        quantization_config=quantization,
        device_map="auto",
        local_files_only=True,
    )
    tokenizer = AutoTokenizer.from_pretrained(str(MODEL_PATH), local_files_only=True)
    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token
    MODEL, TOKENIZER = model, tokenizer
    return MODEL, TOKENIZER


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _in_range(value, lo: int, hi: int) -> bool:
    return _is_number(value) and lo <= value <= hi


def _number(value: float) -> float | int:
    """Keep whole angles as ints so the JSON Dum-E writes stays tidy."""
    return int(value) if float(value).is_integer() else float(value)


def _write_json(path: Path, data: dict) -> None:
    # Write to a sibling temp file and rename, so a crash or a full disk can
    # never leave a half-written robot_state.json behind.
    temp = path.with_name(path.name + ".tmp")
    with temp.open("w", encoding="utf-8") as fp:
        json.dump(data, fp, indent=2)
        fp.write("\n")
        fp.flush()
    temp.replace(path)


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


def _emit(command: dict) -> None:
    absolute = absolute_command(command)
    save_command(command)
    save_state(command)
    print(json.dumps(absolute, separators=(",", ":")))


def load_state() -> dict:
    """The last pose Dum-E remembers, per joint.

    Anything unusable in the file falls back to the default for that joint
    alone, so one bad value never erases the whole remembered pose.
    """
    try:
        stored = json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
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
    """Guarantee robot_state.json exists and holds five concrete angles.

    On a first run this creates it at the default pose, so the file Dum-E
    starts from is always a full set of real numbers rather than nothing or a
    null. An angle that is missing, null or out of range in an existing file is
    replaced by its default; every angle that is still usable is kept, so this
    never throws away what the arm has already been doing.
    """
    state = load_state()
    _write_json(STATE_FILE, state)
    return state


def save_state(command: dict) -> None:
    if command["action"] == "home":
        _write_json(STATE_FILE, DEFAULT_STATE.copy())
        return
    if command["action"] not in MOVE_ACTIONS:
        return
    state = load_state()
    for joint in ALL_JOINTS:
        value = command.get(joint)
        # A null or unusable angle keeps the remembered one, so the pose on
        # disk only ever moves to a value the arm can actually hold.
        if _in_range(value, *JOINT_RANGES[joint]):
            state[joint] = value
    _write_json(STATE_FILE, state)


def resolve_command(action: str, direction: str = "default", overrides: dict | None = None) -> dict:
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
        command.update({joint: overrides[joint] for joint in ALL_JOINTS if joint in overrides})
    return command


def _touches(command: dict, joints) -> bool:
    return any(command.get(joint) is not None for joint in joints)


def coerce_action(action: str, overrides: dict) -> str:
    if action != "turn" or not any(joint in overrides for joint in VERTICAL_JOINTS):
        return action
    return "move"


def is_valid_command(command: object) -> bool:
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
    if action == "turn":
        if command["base"] is None or _touches(command, VERTICAL_JOINTS):
            return False
    if action == "tilt" and command["base"] is not None:
        return False
    if action in MOVE_ACTIONS and not _touches(command, ALL_JOINTS):
        return False
    return True


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
    return bool(MOVE_VERB.search(prompt) or TURN_VERB.search(prompt) or TILT_VERB.search(prompt))


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


def parse_explicit_values(prompt: str) -> dict:
    normalized = _normalize_prompt(prompt)
    overrides: dict = {}
    for match in EXPLICIT_TARGET.finditer(normalized):
        target = JOINT_ALIASES.get(match.group(1).strip().lower(), match.group(1).strip().lower())
        if target not in ALL_JOINTS:
            continue
        value = float(match.group(2))
        if _in_range(value, *JOINT_RANGES[target]):
            overrides[target] = _number(value)
    both = re.search(
        r"\b(?:both|all)\s+joints?\b[^.\n\d-]{0,15}(-?\d+)\b",
        normalized,
        re.IGNORECASE,
    )
    if both:
        value = float(both.group(1))
        for joint in VERTICAL_JOINTS:
            if _in_range(value, *JOINT_RANGES[joint]):
                overrides.setdefault(joint, _number(value))
    return overrides


GRIPPER_NOUN = re.compile(r"\b(?:gripper|fingers?|hand|fist|claw)\b", re.IGNORECASE)
GRIPPER_GESTURE = re.compile(
    r"\b(?:open|unclench|release|let\s+go|drop|close|closed|clench|squeeze|firmly|grip|grab|hold)\b",
    re.IGNORECASE,
)
GRIPPER_AMOUNT = re.compile(
    r"\b(?:gripper|fingers?|hand)\b[^.\n\d-]{0,20}(-?\d+)\b"
    r"|\b(\d+)\s*(?:percent|%)\b",
    re.IGNORECASE,
)


GRIPPER_HALF = re.compile(r"\bhalf(?:way)?\b")
GRIPPER_VALUES = (
    (re.compile(r"\b(?:gently|softly)\b"), 30),
    (re.compile(r"\b(?:open|unclench|release|let\s+go|drop)\b"), 100),
    (re.compile(r"\b(?:close|closed|clench|squeeze|fist|firmly)\b"), 0),
    (re.compile(r"\b(?:grip|grab|hold)\b"), 15),
)


def user_specifies_gripper(prompt: str) -> bool:
    return bool(GRIPPER_NOUN.search(prompt) or GRIPPER_GESTURE.search(prompt))


def gripper_from_prompt(prompt: str) -> float | None:
    normalized = _normalize_prompt(prompt)
    amount = GRIPPER_AMOUNT.search(normalized)
    if amount:
        value = float(next(group for group in amount.groups() if group is not None))
        if _in_range(value, *JOINT_RANGES["gripper"]):
            return _number(value)
    # "halfway" is only a gripper value when the gripper is actually named;
    # the rest of the gestures mean the gripper on their own.
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


def build_local_command(prompt: str) -> dict | None:
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


def ask_bot(prompt: str) -> None:
    local_command = build_local_command(prompt)
    if local_command is not None:
        _emit(local_command)
        return

    print("Dum-E is thinking...")
    try:
        command = _ask_model(prompt)
    except Exception as exc:
        print(f"Dum-E: I hit an error: {exc}")
        print("Falling back to help.")
        command = resolve_command("help")
    _emit(command)


def _ask_model(prompt: str) -> dict:
    import torch

    model, tokenizer = _load_model()
    prompt_text = tokenizer.apply_chat_template(
        [{"role": "system", "content": SYSTEM_PROMPT}, {"role": "user", "content": prompt}],
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


def main() -> None:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")

    print(f"Dum-E ready. Using model: {MODEL_PATH.name}")
    print(HELP_TEXT)
    print("Type 'exit', 'quit', or 'bye' to stop.")

    # Create robot_state.json up front, so the pose Dum-E starts from is a full
    # set of default angles instead of a missing file.
    ensure_state_file()

    while True:
        try:
            user_input = input("\nYou: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nDum-E: Goodbye!")
            break

        if not user_input:
            continue

        if user_input.lower() in {"exit", "quit", "bye"}:
            print("Dum-E: Goodbye!")
            break

        if user_input.lower() == "help":
            print(HELP_TEXT)
            continue

        ask_bot(user_input)


if __name__ == "__main__":
    main()
