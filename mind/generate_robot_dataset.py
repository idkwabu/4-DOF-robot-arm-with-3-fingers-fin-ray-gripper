"""Build Dum-E's intent dataset.

The model only ever chooses an action and a direction. Every angle lives in
mind.DIRECTION_PRESETS, so the targets here are exactly two fields:

    {"action": "...", "direction": "..."}

Two kinds of prompt never belong in this file, because mind.py answers them
without asking the model at all:

- anything naming a direction (up, down, left, right, middle, or a compound)
- anything naming a number (a joint, the base, or a gripper percentage)

The model therefore exists for home, stop, status, help, and for the ambiguous
leftovers that must become help. Direction examples are kept because a prompt
such as "face the middle and turn left" is deliberately deferred to the model.
"""

import json
import random
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import mind

BASE_DIR = Path(__file__).resolve().parent
SYSTEM = (BASE_DIR / "system_prompt.txt").read_text(encoding="utf-8").strip()
TRAIN_FILE = BASE_DIR / "robot_dataset.jsonl"
EVAL_FILE = BASE_DIR / "robot_dataset_eval.jsonl"
ROWS_PER_CLASS = 570
EVAL_FRACTION = 0.1
SEED = 20260926

# (prompt, action, direction)
SEEDS = {
    "turn": [
        ("Turn right", "right"),
        ("Turn the arm to the left", "left"),
        ("Turn left", "left"),
        ("Swivel to the right", "right"),
        ("Turn to the middle", "middle"),
        ("Turn to center", "middle"),
        ("Face middle", "middle"),
        ("Turn the base all the way to the right", "right"),
        ("Spin the base to the left", "left"),
        ("Rotate the base halfway", "middle"),
        ("Pivot to the right", "right"),
        ("Steer the arm left", "left"),
        ("Face the right wall", "right"),
        ("Rotate the base to the middle", "middle"),
        ("Point the base to the middle", "middle"),
        ("Steer the base to the middle", "middle"),
    ],
    "tilt": [
        ("Move up", "up"),
        ("Go upward", "up"),
        ("Tilt up", "up"),
        ("Raise the arm", "up"),
        ("Move down", "down"),
        ("Lower the arm", "down"),
        ("Go down", "down"),
        ("Tilt down slowly", "down"),
        ("Move the arm straight up", "up"),
        ("Put the arm down", "down"),
        ("Raise it higher", "up"),
        ("Lower it further", "down"),
        ("Lift the arm up", "up"),
        ("Dip the arm down", "down"),
    ],
    "move": [
        ("Move up and to the left", "up-left"),
        ("Go up left", "up-left"),
        ("Shift up and left", "up-left"),
        ("Move up and to the right", "up-right"),
        ("Go up right", "up-right"),
        ("Reach up and right", "up-right"),
        ("Move down and to the left", "down-left"),
        ("Lower the arm to the left", "down-left"),
        ("Go down left", "down-left"),
        ("Move down and to the right", "down-right"),
        ("Shift down right", "down-right"),
        ("Go down and right", "down-right"),
        ("Tilt up and turn left", "up-left"),
        ("Raise the arm and swing right", "up-right"),
        ("Lower the arm and turn left", "down-left"),
        ("Point the arm up and to the right", "up-right"),
        ("go up and swing left", "up-left"),
        ("left and up at the same time", "up-left"),
        ("tilt up while turning right", "up-right"),
        ("go down-left", "down-left"),
        ("down and to the right, please", "down-right"),
        ("raise it and turn right", "up-right"),
        ("Raise the arm to the middle", "up-middle"),
        ("Move up and face the middle", "up-middle"),
        ("Go up to the center", "up-middle"),
        ("Lift the arm to the middle", "up-middle"),
        ("Lower the arm to the middle", "down-middle"),
        ("Move down and face the middle", "down-middle"),
        ("Go down to the center", "down-middle"),
        ("Bring the arm down to the middle", "down-middle"),
        ("Tilt down and center the base", "down-middle"),
    ],
    "home": [
        ("Go home", "default"),
        ("Return to the home position", "default"),
        ("Reset the arm", "default"),
        ("Go back to the start position", "default"),
        ("Return home", "default"),
        ("Put the arm back in its default position", "default"),
        ("Move the arm to home", "default"),
        ("Bring it back to the start", "default"),
        ("Home the arm", "default"),
        ("Go to the rest position", "default"),
        ("Restart from the home position", "default"),
        ("Park the arm", "default"),
    ],
    "stop": [
        ("Stop now", "default"),
        ("Emergency stop", "default"),
        ("Freeze", "default"),
        ("Hold still", "default"),
        ("Don't move", "default"),
        ("Stop everything", "default"),
        ("Halt the arm", "default"),
        ("Stop right now", "default"),
        ("Do not move any further", "default"),
        ("Kill the motors", "default"),
        ("Stop where you are", "default"),
    ],
    "status": [
        ("What is the robot doing?", "default"),
        ("Show robot status", "default"),
        ("Report your status", "default"),
        ("What are you doing right now?", "default"),
        ("Give me a status update", "default"),
        ("What is the current state of the arm?", "default"),
        ("Where is the arm right now?", "default"),
        ("Check the arm", "default"),
        ("Any change since last time?", "default"),
        ("Give me a status report", "default"),
        ("What pose is the arm in?", "default"),
        ("Read me the joint angles", "default"),
        ("Log the current angles", "default"),
        ("Confirm your position", "default"),
        ("Is the arm moving?", "default"),
        ("Summarise the last command", "default"),
    ],
    "help": [
        ("What can you do?", "default"),
        ("Do something dangerous", "default"),
        ("Move the arm", "default"),
        ("Tell me a joke", "default"),
        ("What commands do you understand?", "default"),
        ("Help", "default"),
        ("Give me help", "default"),
        ("What are your capabilities?", "default"),
        ("Move the arm up without a distance", "default"),
        ("Hello", "default"),
        ("Who are you?", "default"),
        ("Are you a robot?", "default"),
        ("Open the fridge", "default"),
        ("What is the weather like?", "default"),
        ("Sing me a song", "default"),
        ("I need help", "default"),
        ("Help me", "default"),
        ("What can this arm do?", "default"),
        ("How do I use you?", "default"),
        ("List your commands", "default"),
        ("Do you understand me?", "default"),
        ("Explain the commands", "default"),
        ("Set an alarm", "default"),
        ("Play some music", "default"),
    ],
}

PREFIXES = (
    "",
    "Please ",
    "Dum-E, ",
    "Robot, ",
    "Could you ",
    "I want you to ",
    "I need you to ",
    "Kindly ",
    "Can you ",
    "Would you ",
    "Go ahead and ",
    "Hey Dum-E, ",
)

SUFFIXES = (
    "",
    " now",
    " carefully",
    " slowly",
    " for me",
    " immediately",
    " when ready",
    " as requested",
    " please",
    " thanks",
)


def build_rows(action: str, direction: str, seed_text: str) -> list[str]:
    is_question = seed_text.endswith("?")
    rows = []
    for prefix in PREFIXES:
        body = seed_text
        if prefix and not is_question:
            body = body[0].lower() + body[1:]
        for suffix in SUFFIXES:
            if is_question and suffix:
                continue
            prompt = f"{prefix}{body}{suffix}"
            if prefix in {"Could you ", "Can you ", "Would you "} and not prompt.endswith("?"):
                prompt += "?"
            rows.append(prompt)
    return sorted(set(rows))


def target(action: str, direction: str) -> dict:
    return {"action": action, "direction": direction}


def validate(family: list[tuple[str, dict]]) -> None:
    for prompt, command in family:
        assert isinstance(prompt, str) and prompt.strip(), prompt
        assert sorted(command) == ["action", "direction"], command
        assert command["action"] in mind.ALL_ACTIONS, command
        assert command["direction"] in mind.ALL_DIRECTION_VALUES, command
        # A digit means the prompt names an angle, which only a real preset
        # direction can absorb. "default" is deliberately absent, so this reads
        # the presets rather than the full value set and cannot drift when a
        # direction is added.
        assert not any(char.isdigit() for char in prompt) or command["direction"] in mind.DIRECTION_PRESETS, prompt
        assert command["direction"] == "default" or command["action"] in mind.MOVE_ACTIONS, command


def seeds_for(groups, action: str, direction: str) -> list[str]:
    return groups[(action, direction)]


def split_families(families: list[list[tuple[str, dict]]], rng: random.Random):
    """Hold out whole seed families so no prompt variant straddles the split."""
    ordered = list(families)
    rng.shuffle(ordered)
    holdout = max(1, round(len(ordered) * EVAL_FRACTION))
    return ordered[holdout:], ordered[:holdout]


def write(path: Path, rows: list[tuple[str, dict]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for prompt, command in rows:
            record = {
                "messages": [
                    {"role": "system", "content": SYSTEM},
                    {"role": "user", "content": prompt},
                    {"role": "assistant", "content": json.dumps(command, separators=(",", ":"))},
                ]
            }
            handle.write(json.dumps(record, ensure_ascii=True) + "\n")


def main() -> int:
    rng = random.Random(SEED)
    train_rows: list[tuple[str, dict]] = []
    eval_rows: list[tuple[str, dict]] = []

    groups: dict[tuple[str, str], list[str]] = defaultdict(list)
    for action, entries in SEEDS.items():
        for seed_text, direction in entries:
            groups[(action, direction)].append(seed_text)

    for (action, direction), seed_texts in sorted(groups.items()):
        command = target(action, direction)
        families = []
        for seed_text in seed_texts:
            family = [(prompt, command) for prompt in build_rows(action, direction, seed_text)]
            validate(family)
            families.append(family)
        train_families, eval_families = split_families(families, rng)
        for family in train_families:
            train_rows.extend(family)
        for family in eval_families:
            eval_rows.extend(family)
        print(
            f"{action:6} {direction:10} {len(seeds_for(groups, action, direction)):2} seeds"
            f" -> {len(train_families):2} train, {len(eval_families):2} holdout"
        )

    rng.shuffle(train_rows)
    rng.shuffle(eval_rows)

    per_class = defaultdict(int)
    balanced_train = []
    for prompt, command in train_rows:
        if per_class[command["action"]] >= ROWS_PER_CLASS:
            continue
        per_class[command["action"]] += 1
        balanced_train.append((prompt, command))

    write(TRAIN_FILE, balanced_train)
    write(EVAL_FILE, eval_rows)

    prompts = {prompt for prompt, _ in balanced_train} & {prompt for prompt, _ in eval_rows}
    directions = {(c["action"], c["direction"]) for _, c in balanced_train}
    print(f"\nwrote {len(balanced_train)} train rows to {TRAIN_FILE.name}")
    print(f"wrote {len(eval_rows)} holdout rows to {EVAL_FILE.name}")
    print(f"train prompts shared with holdout: {len(prompts)}")
    print(f"distinct action/direction pairs in train: {len(directions)}")
    print("per class:", dict(sorted(per_class.items())))

    assert not prompts, "holdout leaked into training"
    assert len(balanced_train) >= 3000, "training set is too small"
    assert len(eval_rows) >= 300, "holdout is too small to measure anything"
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
