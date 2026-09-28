import contextlib
import io
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mind

FAILURES = []
CHECKS = 0


def check(label, actual, expected):
    global CHECKS
    CHECKS += 1
    if actual != expected:
        FAILURES.append(f"{label}\n     expected: {expected}\n     actual:   {actual}")


def check_true(label, value):
    check(label, bool(value), True)


VERTICAL = ("joint1", "joint2", "joint3")
EXPECTED_PRESETS = {
    "up": {"base": None, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": None},
    "down": {"base": None, "joint1": 60, "joint2": -90, "joint3": -60, "gripper": None},
    "left": {"base": 135, "joint1": None, "joint2": None, "joint3": None, "gripper": None},
    "right": {"base": 45, "joint1": None, "joint2": None, "joint3": None, "gripper": None},
    "hard-left": {"base": 180, "joint1": None, "joint2": None, "joint3": None, "gripper": None},
    "hard-right": {"base": 0, "joint1": None, "joint2": None, "joint3": None, "gripper": None},
    "up-left": {"base": 135, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": None},
    "up-right": {"base": 45, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": None},
    "down-left": {"base": 135, "joint1": 60, "joint2": -90, "joint3": -60, "gripper": None},
    "down-right": {"base": 45, "joint1": 60, "joint2": -90, "joint3": -60, "gripper": None},
    "middle": {"base": 90, "joint1": None, "joint2": None, "joint3": None, "gripper": None},
    "up-middle": {"base": 90, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": None},
    "down-middle": {"base": 90, "joint1": 60, "joint2": -90, "joint3": -60, "gripper": None},
}
ANGLE_KEYS = ("base", "joint1", "joint2", "joint3", "gripper")


def angles(command):
    return {key: command[key] for key in ANGLE_KEYS}


def test_tables_are_coherent():
    check("preset keys match axes for every direction", sorted(mind.DIRECTION_PRESETS), sorted(mind.DIRECTION_AXES))
    for direction, preset in mind.DIRECTION_PRESETS.items():
        check(f"preset {direction} touches only its axes", set(preset) - set(mind.DIRECTION_AXES[direction]), set())
    for direction, expected in EXPECTED_PRESETS.items():
        resolved = mind.resolve_command("move", direction)
        check(f"resolve {direction}", angles(resolved), expected)
        check_true(f"resolve {direction} is valid", mind.is_valid_command(resolved))
    check("angle table matches the spec", {d: angles(mind.resolve_command("move", d)) for d in EXPECTED_PRESETS}, EXPECTED_PRESETS)


def test_all_nineteen_intents():
    cases = [
        ("turn", "left"), ("turn", "right"), ("turn", "middle"), ("turn", "default"),
        ("tilt", "up"), ("tilt", "down"),
        ("move", "left"), ("move", "right"),
        ("move", "up-left"), ("move", "up-right"),
        ("move", "down-left"), ("move", "down-right"),
        ("move", "up-middle"), ("move", "down-middle"), ("move", "default"),
        ("home", "default"), ("stop", "default"),
        ("status", "default"), ("help", "default"),
    ]
    check("nineteen canonical intents", len(cases), 19)
    for action, direction in cases:
        command = mind.resolve_command(action, direction)
        check(f"{action}/{direction} has 7 fields", sorted(command), sorted(mind.COMMAND_KEYS))
        if action == "move" and direction == "default":
            check_true("a bare move/default is a no-op and invalid", not mind.is_valid_command(command))
            continue
        check_true(f"{action}/{direction} valid", mind.is_valid_command(command))
        if action == "home":
            check("home is fully concrete", angles(command), dict(mind.DEFAULT_STATE))
        elif action in {"stop", "status", "help"}:
            check(f"{action} moves nothing", angles(command), dict.fromkeys(ANGLE_KEYS))
        elif action == "turn" and direction == "default":
            check("turn middle points base at 90", angles(command)["base"], 90)
        if action == "turn":
            check_true(f"turn/{direction} keeps every joint null", all(command[j] is None for j in VERTICAL + ("gripper",)))
        if action == "tilt":
            check(f"tilt/{direction} leaves base alone", command["base"], None)


def test_no_op_is_rejected():
    check_true("a movement with nothing to move is invalid", not mind.is_valid_command(mind.resolve_command("move", "default")))


def test_overrides_win_over_presets():
    command = mind.resolve_command("move", "left", {"base": 45, "gripper": 100})
    check("explicit base overrides the left preset", angles(command),
          {"base": 45, "joint1": None, "joint2": None, "joint3": None, "gripper": 100})
    check_true("overridden left is still valid", mind.is_valid_command(command))
    check("turn plus a vertical joint becomes move", mind.coerce_action("turn", {"joint2": 30}), "move")
    check("turn without a vertical joint stays turn", mind.coerce_action("turn", {"base": 10}), "turn")
    check("tilt is never coerced", mind.coerce_action("tilt", {"joint2": 30}), "tilt")


def test_validation_rejects():
    def mutate(**changes):
        command = mind.resolve_command("move", "up-left")
        command.update(changes)
        return command

    check_true("joint2 above its range is rejected", not mind.is_valid_command(mutate(joint2=200)))
    check_true("negative base is rejected", not mind.is_valid_command(mutate(base=-5)))
    check_true("gripper above 100 is rejected", not mind.is_valid_command(mutate(gripper=150)))
    check_true("joint1 above 180 is rejected", not mind.is_valid_command(mutate(joint1=181)))
    check_true("a missing field is rejected", not mind.is_valid_command({"action": "move", "direction": "up"}))
    check_true("an extra field is rejected", not mind.is_valid_command({**mind.resolve_command("move", "up"), "speed": 1}))
    check_true("an unknown action is rejected", not mind.is_valid_command(mutate(action="fly")))
    check_true("an unknown direction is rejected", not mind.is_valid_command(mutate(direction="sideways")))
    check_true("turn up is incoherent", not mind.is_valid_command(mind.resolve_command("turn", "up")))
    check_true("tilt left is incoherent", not mind.is_valid_command(mind.resolve_command("tilt", "left")))
    check_true("home with a direction is rejected", not mind.is_valid_command(mind.resolve_command("home", "left")))
    check_true("help with a direction is rejected", not mind.is_valid_command(mind.resolve_command("help", "up")))
    check_true("up may not move the base", not mind.is_valid_command({**mind.resolve_command("move", "up"), "base": 45}))
    check_true("down may not move the base", not mind.is_valid_command({**mind.resolve_command("move", "down"), "base": 45}))
    check_true("left may not move the joints", not mind.is_valid_command({**mind.resolve_command("move", "left"), "joint2": 10}))
    check_true("a turn may still open the gripper", mind.is_valid_command(mind.resolve_command("turn", "left", {"gripper": 100})))
    check_true("a bool is not an angle", not mind.is_valid_command(mutate(base=True)))
    check_true("a string is not an angle", not mind.is_valid_command(mutate(base="180")))


def test_local_parser():
    cases = [
        ("turn left", {"action": "turn", "direction": "left", "base": 135, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("Turn the base all the way to the left", {"action": "turn", "direction": "hard-left", "base": 180, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("swivel to the right", {"action": "turn", "direction": "right", "base": 45, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("tilt up", {"action": "tilt", "direction": "up", "base": None, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": None}),
        ("lower the arm", {"action": "tilt", "direction": "down", "base": None, "joint1": 60, "joint2": -90, "joint3": -60, "gripper": None}),
        ("move up and to the left", {"action": "move", "direction": "up-left", "base": 135, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": None}),
        ("Go down right", {"action": "move", "direction": "down-right", "base": 45, "joint1": 60, "joint2": -90, "joint3": -60, "gripper": None}),
        ("open the gripper", {"action": "move", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": 100}),
        ("close the hand", {"action": "move", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": 0}),
        ("open the gripper halfway", {"action": "move", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": 50}),
        ("Open the fingers 75 percent", {"action": "move", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": 75}),
        ("rotate the base to ninety degrees", {"action": "turn", "direction": "default", "base": 90, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("Move the base to 135", {"action": "turn", "direction": "default", "base": 135, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("set joint two to minus sixty", {"action": "move", "direction": "default", "base": None, "joint1": None, "joint2": -60, "joint3": None, "gripper": None}),
        ("Put J1 at 120, J2 at 30, and open the fingers", {"action": "move", "direction": "default", "base": None, "joint1": 120, "joint2": 30, "joint3": None, "gripper": 100}),
        ("set both joints to zero", {"action": "move", "direction": "default", "base": None, "joint1": 0, "joint2": 0, "joint3": 0, "gripper": None}),
        ("turn left and open the gripper", {"action": "turn", "direction": "left", "base": 135, "joint1": None, "joint2": None, "joint3": None, "gripper": 100}),
        ("stop", {"action": "stop", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("Freeze", {"action": "stop", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("Grip firmly", {"action": "move", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": 0}),
        ("Release the object", {"action": "move", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": 100}),
        ("Let go", {"action": "move", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": 100}),
        ("Hold on", {"action": "move", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": 15}),
        ("Squeeze gently", {"action": "move", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": 30}),
        ("Make a fist", {"action": "move", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": 0}),
        ("Rotate the base halfway", {"action": "turn", "direction": "middle", "base": 90, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("Go ahead and turn left", {"action": "turn", "direction": "left", "base": 135, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("Go ahead and raise the arm", {"action": "tilt", "direction": "up", "base": None, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": None}),
        ("Go ahead and move up-left", {"action": "move", "direction": "up-left", "base": 135, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": None}),
        ("I need you to stop everything immediately", {"action": "stop", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("move straight ahead", {"action": "turn", "direction": "middle", "base": 90, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("lower the arm to the left", {"action": "move", "direction": "down-left", "base": 135, "joint1": 60, "joint2": -90, "joint3": -60, "gripper": None}),
        ("raise it and turn right", {"action": "move", "direction": "up-right", "base": 45, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": None}),
        ("turn left and raise the arm", {"action": "move", "direction": "up-left", "base": 135, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": None}),
        ("push it down and to the right", {"action": "move", "direction": "down-right", "base": 45, "joint1": 60, "joint2": -90, "joint3": -60, "gripper": None}),
        ("lift the arm and swing left", {"action": "move", "direction": "up-left", "base": 135, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": None}),
        ("Rotate the base to one hundred and thirty five degrees", {"action": "turn", "direction": "default", "base": 135, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("Turn the base to one hundred twenty", {"action": "turn", "direction": "default", "base": 120, "joint1": None, "joint2": None, "joint3": None, "gripper": None}),
        ("Set joint three to minus thirty and close the gripper", {"action": "move", "direction": "default", "base": None, "joint1": None, "joint2": None, "joint3": -30, "gripper": 0}),
        ("Move the first joint to 180 and the second to 90, fingers 25 percent open", {"action": "move", "direction": "default", "base": None, "joint1": 180, "joint2": 90, "joint3": None, "gripper": 25}),
        ("Move the arm to base 45, joint1 120, joint2 30, gripper 50", {"action": "move", "direction": "default", "base": 45, "joint1": 120, "joint2": 30, "joint3": None, "gripper": 50}),
    ]
    for prompt, expected in cases:
        command = mind.build_local_command(prompt)
        check(f"local: {prompt!r}", None if command is None else {"action": command["action"], "direction": command["direction"], **angles(command)}, expected)
        if command is not None:
            check_true(f"local result is valid: {prompt!r}", mind.is_valid_command(command))
    check("a joke defers to the model", mind.build_local_command("tell me a joke"), None)
    # The four control actions are settled by keyword, so they no longer wait
    # on the model. Only genuinely ambiguous wording still defers.
    for prompt, action in (("go home", "home"),
                           ("return to the home position", "home"),
                           ("where is the arm right now?", "status"),
                           ("go ahead and put the arm back in its default position", "home"),
                           ("what can you do", "help"),
                           ("go ahead and help now", "help")):
        command = mind.build_local_command(prompt)
        check(f"control request is answered locally: {prompt!r}",
              None if command is None else command["action"], action)
    for prompt in ("someone told me ten things", "one moment please", "hello there",
                   "don't stop", "do not stop moving",
                   "don't go home yet", "never go home"):
        check(f"ambiguous request defers to the model: {prompt!r}", mind.build_local_command(prompt), None)


def test_hard_turns():
    """An intensifier turns a 45 degree turn into the full 90 to the stop."""
    for prompt in ("turn all the way to the left", "all the way left",
                   "as far as you can to the left", "turn as far as you can left",
                   "turn fully left", "turn completely left", "turn entirely to the left",
                   "turn hard left", "turn far left", "turn max left", "turn maximum left",
                   "turn the base maximally leftward", "hard left"):
        command = mind.build_local_command(prompt)
        check(f"hard-left is answered locally: {prompt!r}",
              None if command is None else (command["action"], command["direction"]), ("turn", "hard-left"))
        check(f"hard-left reaches the stop: {prompt!r}",
              None if command is None else command["base"], 180)
    for prompt in ("turn all the way to the right", "all the way right",
                   "as far as you can to the right", "turn fully right",
                   "turn completely right", "turn entirely rightward",
                   "turn hard right", "turn far right", "turn max right"):
        command = mind.build_local_command(prompt)
        check(f"hard-right is answered locally: {prompt!r}",
              None if command is None else (command["action"], command["direction"]), ("turn", "hard-right"))
        check(f"hard-right reaches the stop: {prompt!r}",
              None if command is None else command["base"], 0)

    # A plain turn stays soft no matter how it is dressed up.
    for prompt in ("turn left", "turn to the left", "swivel to the left",
                   "turn right", "turn to the right", "swivel to the right"):
        command = mind.build_local_command(prompt)
        check(f"a plain turn is still soft: {prompt!r}",
              None if command is None else (command["direction"], command["base"]),
              ("left", 135) if "left" in prompt else ("right", 45))

    # Hardening is a base-only move, so an intensifier on a diagonal is ignored
    # rather than swinging the arm further than the plain turn it is named after.
    for prompt, direction in (("move up and all the way to the left", "up-left"),
                              ("lower the arm all the way to the left", "down-left"),
                              ("raise it and turn all the way right", "up-right"),
                              ("push it down and completely to the right", "down-right")):
        command = mind.build_local_command(prompt)
        check(f"an intensifier does not harden a diagonal: {prompt!r}",
              None if command is None else (command["action"], command["direction"]),
              ("move", direction))
        check(f"the diagonal keeps the soft base: {prompt!r}",
              None if command is None else command["base"],
              135 if direction.endswith("left") else 45)

    # An intensifier that does not modify a turn leaves the turn soft.
    for prompt in ("the arm is fully extended, turn left",
                   "turn left after checking the camera is completely calibrated",
                   "the gripper is fully closed, now turn left"):
        command = mind.build_local_command(prompt)
        check(f"a stray intensifier does not harden: {prompt!r}",
              None if command is None else (command["direction"], command["base"]), ("left", 135))

    # A sentence that merely contains the phrase is not a turn command.
    for prompt in ("as far as the left wheel goes, replace the belt",
                   "as far as you can see, the left joint is at 45 degrees",
                   "the left side is all the way up"):
        check(f"a non-command defers to the model: {prompt!r}", mind.build_local_command(prompt), None)

    check("hard-left touches only the base", sorted(mind.DIRECTION_AXES["hard-left"]), ["base"])
    check("hard-right touches only the base", sorted(mind.DIRECTION_AXES["hard-right"]), ["base"])
    check_true("a hard turn is a valid turn", mind.is_valid_command(mind.resolve_command("turn", "hard-left")))
    check_true("a hard turn is a valid turn the other way", mind.is_valid_command(mind.resolve_command("turn", "hard-right")))
    check("resolve hard-left", mind.resolve_command("turn", "hard-left")["base"], 180)
    check("resolve hard-right", mind.resolve_command("turn", "hard-right")["base"], 0)
    check("a hard turn is not a no-op", mind.is_valid_command(mind.resolve_command("turn", "hard-left")), True)


def test_bare_middle():
    """A middle word on its own centers the base, with no verb needed.

    "middle" names no turn verb, so before _is_bare_middle these fell through
    to the model. The model has no bare-middle training row, answered "did not
    return an action and a direction", and the resolver fell back to help --
    which moved every joint instead of just the base.

    middle is now a direction in its own right rather than a base-90 default,
    which is what lets the model be trained on it and lets the axis checks
    reject a command that tries to steer the arm while centering.
    """
    centered = {"action": "turn", "direction": "middle", "base": 90,
                "joint1": None, "joint2": None, "joint3": None, "gripper": None}
    for prompt in ("middle", "center", "centre", "straight", "straight ahead",
                   "halfway", "ahead", "middle please", "return to the middle",
                   "point the base at the middle", "just center",
                   "straight ahead please", "halfway to the middle"):
        command = mind.build_local_command(prompt)
        check(f"bare middle is answered locally: {prompt!r}",
              None if command is None else {"action": command["action"],
                                           "direction": command["direction"],
                                           **angles(command)}, centered)
        if command is not None:
            check_true(f"bare middle result is valid: {prompt!r}",
                       mind.is_valid_command(command))

    # Centering is a base-only move, so the arm joints must be left out
    # entirely rather than sent to their defaults the way help would.
    command = mind.build_local_command("middle")
    check("bare middle touches only the base",
          [key for key in ANGLE_KEYS if command[key] is not None], ["base"])
    check("bare middle sends no joint angles", [command[key] for key in ANGLE_KEYS[1:]],
          [None, None, None, None])
    check_true("bare middle is a valid turn", mind.is_valid_command(command))
    check("bare middle is not a no-op", mind.is_valid_command(
        mind.resolve_command("turn", "default")), True)
    check("resolve default centers the base", mind.resolve_command("turn", "default")["base"], 90)

    # Naming a turn verb still reaches the same place, and the old spellings
    # are unchanged by this.
    for prompt in ("go to the middle", "turn middle", "turn to the middle",
                   "move straight ahead", "bring it back to the middle",
                   "rotate the base to the middle"):
        command = mind.build_local_command(prompt)
        check(f"the verb'd form is unchanged: {prompt!r}",
              None if command is None else {"action": command["action"],
                                           "direction": command["direction"],
                                           **angles(command)}, centered)

    # A middle word inside a longer sentence is not a command. These have to
    # keep reaching the model, which is the only thing that can read them.
    for prompt in ("center of gravity", "the middle of the road",
                   "replace the middle gearbox", "the center of the target is off",
                   "align the middle segment of the arm", "center of the arena please"):
        check(f"a non-command defers to the model: {prompt!r}", mind.build_local_command(prompt), None)

    # "go ahead" is deliberately excluded from DIRECTION_MIDDLE, so the filler
    # set can never turn it into a command.
    check("go ahead is not a middle command", mind.DIRECTION_MIDDLE.search("go ahead"), None)
    check("go ahead defers to the model", mind.build_local_command("go ahead"), None)
    check_true("go ahead and turn left is still a turn",
               (lambda c: c is not None and c["direction"] == "left" and c["base"] == 135)(
                   mind.build_local_command("go ahead and turn left")))

    # A real direction alongside a middle word keeps deferring to the model, as
    # it did before: "turn left to the middle" is ambiguous enough that only the
    # model should decide, so centering must not silently claim it.
    for prompt in ("turn left to the middle", "turn right toward the middle",
                   "turn all the way left past the middle"):
        check(f"a real direction with a middle word still defers: {prompt!r}",
              mind.build_local_command(prompt), None)


def test_middle_compound_directions():
    """up-middle and down-middle pair a centered base with the up/down tilt."""
    up = {"action": "move", "direction": "up-middle", "base": 90,
          "joint1": 120, "joint2": -60, "joint3": -30, "gripper": None}
    down = {"action": "move", "direction": "down-middle", "base": 90,
            "joint1": 60, "joint2": -90, "joint3": -60, "gripper": None}
    for prompt in ("up middle", "raise the arm to the middle", "move up and face the middle",
                   "go up to the center", "lift the arm to the middle", "point up to the middle"):
        command = mind.build_local_command(prompt)
        check(f"up-middle is answered locally: {prompt!r}",
              None if command is None else {"action": command["action"],
                                           "direction": command["direction"],
                                           **angles(command)}, up)
    for prompt in ("down middle", "lower the arm to the middle", "move down and face the middle",
                   "go down to the center", "bring the arm down to the middle",
                   "tilt down and center the base", "point down to the middle"):
        command = mind.build_local_command(prompt)
        check(f"down-middle is answered locally: {prompt!r}",
              None if command is None else {"action": command["action"],
                                           "direction": command["direction"],
                                           **angles(command)}, down)

    # The base is centered in both, so it must land on 90 rather than on the
    # 135/45 the left and right compounds use.
    for direction, expected in (("up-middle", 90), ("down-middle", 90)):
        check(f"{direction} centers the base",
              mind.resolve_command("move", direction)["base"], expected)

    # Only the arm moves; the base and gripper are not touched by the tilt.
    for direction in ("up-middle", "down-middle"):
        command = mind.build_local_command("up middle" if direction == "up-middle" else "down middle")
        check(f"{direction} leaves the gripper alone", command["gripper"], None)
        check_true(f"{direction} is valid", mind.is_valid_command(command))
        check_true(f"{direction} is not a no-op", not (command["base"] == 90
                    and all(command[key] is None for key in ANGLE_KEYS[1:])))

    # An intensifier must not swing a centered base off its middle, the same way
    # it never hardens a diagonal.
    for prompt in ("hard middle", "all the way to the middle", "turn hard to the center"):
        command = mind.build_local_command(prompt)
        if command is not None:
            check_true(f"an intensifier never hardens a middle: {prompt!r}",
                       command["direction"] in mind.MIDDLE_DIRECTIONS)

    # middle steers the base only, so a command that aims the arm as well is
    # rejected exactly as "turn left with joint1 45" is.
    invalid = mind.resolve_command("turn", "middle")
    invalid["joint1"] = 45
    check_true("a middle turn cannot also steer the arm", not mind.is_valid_command(invalid))
    check_true("the compounds may steer the arm",
               mind.is_valid_command(mind.resolve_command("move", "up-middle")))

    for word in ("middle", "center", "centre", "straight", "halfway", "ahead"):
        check(f"the bare middle word is itself allowed: {word!r}", word in mind.MIDDLE_BARE_FILLER, True)
    for word in ("gravity", "road", "gearbox", "arena", "segment", "of", "what", "weather"):
        check_true(f"a content word is not filler: {word!r}", word not in mind.MIDDLE_BARE_FILLER)


def test_intent_parsing():
    check("plain intent", mind.parse_intent('{"action":"tilt","direction":"down"}'), ("tilt", "down"))
    check("spaced intent", mind.parse_intent('{"action": "move", "direction": "up-left"}'), ("move", "up-left"))
    check("intent after a thinking preamble", mind.parse_intent('<think>hmm</think>\n{"action":"turn","direction":"left"}'), ("turn", "left"))
    check("intent without a direction defaults", mind.parse_intent('{"action":"home"}'), ("home", "default"))
    check("prose is not an intent", mind.parse_intent("I cannot do that."), None)
    check("an unknown action is not an intent", mind.parse_intent('{"action":"fly","direction":"up"}'), None)
    check("an unknown direction is not an intent", mind.parse_intent('{"action":"move","direction":"sideways"}'), None)


def test_state_handling():
    with tempfile.TemporaryDirectory() as tmp:
        original = mind.STATE_FILE
        mind.STATE_FILE = Path(tmp) / "robot_state.json"
        try:
            mind._write_json(mind.STATE_FILE, {"base": 45, "joint1": 135, "joint2": 30, "joint3": 30, "gripper": 25})
            check("state loads", mind.load_state(), {"base": 45, "joint1": 135, "joint2": 30, "joint3": 30, "gripper": 25})

            mind.save_state(mind.resolve_command("turn", "left"))
            check("turn left only moves the base", mind.load_state(), {"base": 135, "joint1": 135, "joint2": 30, "joint3": 30, "gripper": 25})

            mind.save_state(mind.resolve_command("tilt", "up"))
            check("tilt up keeps the base and gripper", mind.load_state(), {"base": 135, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": 25})

            mind.save_state(mind.resolve_command("move", "default", {"gripper": 100}))
            check("a gripper move leaves the angles alone", mind.load_state(), {"base": 135, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": 100})

            mind.save_state(mind.resolve_command("stop"))
            check("stop never moves the arm", mind.load_state(), {"base": 135, "joint1": 120, "joint2": -60, "joint3": -30, "gripper": 100})

            mind.save_state(mind.resolve_command("home"))
            check("home resets everything", mind.load_state(), dict(mind.DEFAULT_STATE))

            mind._write_json(mind.STATE_FILE, {"base": None, "joint1": 90, "joint2": -60, "joint3": -60, "gripper": 50})
            check("a null in the state file falls back to defaults", mind.load_state(), dict(mind.DEFAULT_STATE))

            mind._write_json(mind.STATE_FILE, {"base": 900, "joint1": 90, "joint2": -60, "joint3": -60, "gripper": 50})
            check("an out of range state falls back to defaults", mind.load_state(), dict(mind.DEFAULT_STATE))

            mind._write_json(mind.STATE_FILE, {"base": 135, "joint1": "high", "joint2": 30, "joint3": -90, "gripper": 25})
            check("one bad joint does not erase the rest of the pose", mind.load_state(),
                  {"base": 135, "joint1": 90, "joint2": 30, "joint3": -90, "gripper": 25})

            mind._write_json(mind.STATE_FILE, {"base": 0, "joint2": 45})
            check("absent keys default one joint at a time", mind.load_state(),
                  {"base": 0, "joint1": 90, "joint2": 45, "joint3": -60, "gripper": 50})

            mind.STATE_FILE.write_text("[1, 2, 3]", encoding="utf-8")
            check("a state file that is not an object falls back to defaults", mind.load_state(), dict(mind.DEFAULT_STATE))

            mind._write_json(mind.STATE_FILE, dict(mind.DEFAULT_STATE))
            check_true("an atomic write leaves no temp file behind",
                       not (Path(tmp) / "robot_state.json.tmp").exists())
            check("the written file reads back as the pose we gave it", mind.load_state(), dict(mind.DEFAULT_STATE))

            mind.STATE_FILE.write_text("{ not json", encoding="utf-8")
            check("a corrupt state file falls back to defaults", mind.load_state(), dict(mind.DEFAULT_STATE))

            check("a missing state file falls back to defaults", mind.load_state(), dict(mind.DEFAULT_STATE))
        finally:
            mind.STATE_FILE = original

    shipped = json.loads(mind.STATE_FILE.read_text(encoding="utf-8"))
    check_true("the shipped robot_state.json has no nulls",
               set(shipped) == set(mind.ALL_JOINTS) and all(v is not None for v in shipped.values()))
    check_true("every shipped angle is a real in range number",
               all(mind._in_range(shipped[j], *mind.JOINT_RANGES[j]) for j in mind.ALL_JOINTS))


def test_the_first_state_is_never_null():
    with tempfile.TemporaryDirectory() as tmp:
        original = mind.STATE_FILE
        mind.STATE_FILE = Path(tmp) / "robot_state.json"
        try:
            check_true("no state file exists yet", not mind.STATE_FILE.exists())

            # starting up with nothing on disk must create a real pose
            created = mind.ensure_state_file()
            check_true("the first state file is created at startup", mind.STATE_FILE.exists())
            check("the first state is the default pose", created, dict(mind.DEFAULT_STATE))
            check_true("the first state has no nulls",
                       all(v is not None for v in created.values()))
            check("the first state on disk matches", mind.load_state(), dict(mind.DEFAULT_STATE))

            # an existing pose is remembered, not reset
            mind.save_state(mind.resolve_command("tilt", "up"))
            remembered = mind.load_state()
            mind.ensure_state_file()
            check("starting up again keeps the remembered pose", mind.load_state(), remembered)

            # a file full of nulls is repaired, never passed through
            mind._write_json(mind.STATE_FILE, {joint: None for joint in mind.ALL_JOINTS})
            repaired = mind.ensure_state_file()
            check("a state file of nulls is repaired to defaults", repaired, dict(mind.DEFAULT_STATE))
            check_true("the repaired file has no nulls",
                       all(v is not None for v in
                           json.loads(mind.STATE_FILE.read_text(encoding="utf-8")).values()))

            # no action may ever leave a null behind
            for action in sorted(mind.ALL_ACTIONS):
                for direction in sorted(mind.ALL_DIRECTION_VALUES):
                    mind.save_state(mind.resolve_command(action, direction))
                    stored = json.loads(mind.STATE_FILE.read_text(encoding="utf-8"))
                    check_true(f"no null after {action}/{direction}",
                               all(v is not None for v in stored.values()))
                    check_true(f"every angle is in range after {action}/{direction}",
                               all(mind._in_range(stored[j], *mind.JOINT_RANGES[j])
                                   for j in mind.ALL_JOINTS))

            # a command carrying nulls or junk must not poison the memory
            mind._write_json(mind.STATE_FILE, dict(mind.DEFAULT_STATE))
            mind.save_state({"action": "move", "direction": "default", "base": None,
                             "joint1": None, "joint2": 9999, "joint3": None, "gripper": None})
            check("nulls and out of range angles are refused", mind.load_state(),
                  {"base": 90, "joint1": 90, "joint2": -60, "joint3": -60, "gripper": 50})
        finally:
            mind.STATE_FILE = original


def test_state_survives_a_realistic_session():
    with tempfile.TemporaryDirectory() as tmp:
        original = mind.STATE_FILE
        mind.STATE_FILE = Path(tmp) / "robot_state.json"
        try:
            mind.save_state(mind.resolve_command("home"))
            script = ["tilt up", "turn left", "open the gripper", "move down-right", "turn right"]
            for prompt in script:
                command = mind.build_local_command(prompt)
                check_true(f"session command is valid: {prompt!r}", command is not None and mind.is_valid_command(command))
                mind.save_state(command)
            check("the arm never loses a joint to null", mind.load_state(), {"base": 45, "joint1": 60, "joint2": -90, "joint3": -60, "gripper": 100})
            check("every state value is a real number", all(mind._in_range(mind.load_state()[j], *mind.JOINT_RANGES[j]) for j in mind.ALL_JOINTS), True)
        finally:
            mind.STATE_FILE = original


def test_command_file_never_holds_a_null():
    """Whatever the command, the file the arm reads is always a full pose."""
    with tempfile.TemporaryDirectory() as tmp:
        original_state, original_command = mind.STATE_FILE, mind.COMMAND_FILE
        mind.STATE_FILE = Path(tmp) / "robot_state.json"
        mind.COMMAND_FILE = Path(tmp) / "robot_command.json"
        try:
            mind._write_json(mind.STATE_FILE, {"base": 15, "joint1": 30, "joint2": 45,
                                              "joint3": -75, "gripper": 20})
            for action in sorted(mind.ALL_ACTIONS):
                for direction in sorted(mind.ALL_DIRECTION_VALUES):
                    command = mind.resolve_command(action, direction)
                    if not mind.is_valid_command(command):
                        continue
                    mind.save_command(command)
                    written = json.loads(mind.COMMAND_FILE.read_text(encoding="utf-8"))
                    label = f"{action}/{direction}"
                    check_true(f"command file holds every joint after {label}",
                               all(written.get(j) is not None for j in mind.ALL_JOINTS))
                    check_true(f"command file angles are in range after {label}",
                               all(mind._in_range(written[j], *mind.JOINT_RANGES[j])
                                   for j in mind.ALL_JOINTS))
                    check_true(f"command file has no null anywhere after {label}",
                               "null" not in mind.COMMAND_FILE.read_text(encoding="utf-8"))
            # a remembered pose fills in whatever the command left out
            mind._write_json(mind.STATE_FILE, {"base": 15, "joint1": 30, "joint2": 45,
                                              "joint3": -75, "gripper": 20})
            mind.save_command(mind.resolve_command("turn", "left"))
            check("an untouched joint keeps the remembered angle",
                  json.loads(mind.COMMAND_FILE.read_text(encoding="utf-8")),
                  {"action": "turn", "direction": "left", "base": 135, "joint1": 30,
                   "joint2": 45, "joint3": -75, "gripper": 20})
        finally:
            mind.STATE_FILE, mind.COMMAND_FILE = original_state, original_command


def test_the_first_command_starts_from_home():
    """With no memory at all, the first pose the arm is given is home."""
    with tempfile.TemporaryDirectory() as tmp:
        original_state, original_command = mind.STATE_FILE, mind.COMMAND_FILE
        mind.STATE_FILE = Path(tmp) / "robot_state.json"
        mind.COMMAND_FILE = Path(tmp) / "robot_command.json"
        try:
            check("the first run starts with no state file", mind.STATE_FILE.exists(), False)
            started = mind.ensure_state_file()
            check("the first state is the home pose", started, dict(mind.DEFAULT_STATE))
            check("every first angle is a real number",
                  all(mind._in_range(started[j], *mind.JOINT_RANGES[j]) for j in mind.ALL_JOINTS),
                  True)
            # a part-specified move resolves against that home pose
            mind.save_command(mind.resolve_command("turn", "right"))
            check("the first command inherits home for the joints it omits",
                  json.loads(mind.COMMAND_FILE.read_text(encoding="utf-8")),
                  {"action": "turn", "direction": "right", "base": 45, "joint1": 90,
                   "joint2": -60, "joint3": -60, "gripper": 50})
            # a corrupt memory still yields a complete pose, never a null
            mind._write_json(mind.STATE_FILE, "{not json at all")
            mind.save_command(mind.resolve_command("move", "up"))
            written = json.loads(mind.COMMAND_FILE.read_text(encoding="utf-8"))
            check("a corrupt memory falls back to a full home pose",
                  [written[j] for j in mind.ALL_JOINTS],
                  [mind.DEFAULT_STATE[j] if written[j] is None else written[j]
                   for j in mind.ALL_JOINTS])
            check_true("a corrupt memory still writes no null",
                       all(written[j] is not None for j in mind.ALL_JOINTS))
        finally:
            mind.STATE_FILE, mind.COMMAND_FILE = original_state, original_command


def test_control_commands_never_reach_the_model():
    """The whole point of the control words: no weights, no latency."""
    with tempfile.TemporaryDirectory() as tmp:
        original_state = mind.STATE_FILE
        original_command = mind.COMMAND_FILE
        original_load = mind._load_model
        original_ask = mind._ask_model
        mind.STATE_FILE = Path(tmp) / "robot_state.json"
        mind.COMMAND_FILE = Path(tmp) / "robot_command.json"

        def forbidden(*args, **kwargs):
            raise AssertionError("the model was loaded for a control command")

        mind._load_model = forbidden
        mind._ask_model = forbidden
        try:
            for prompt in ("go home", "home", "status", "where are you", "help",
                           "what can you do", "list your commands", "stop", "freeze",
                           "go to the rest position", "is the arm moving"):
                with contextlib.redirect_stdout(io.StringIO()):
                    mind.ask_bot(prompt)
                written = json.loads(mind.COMMAND_FILE.read_text(encoding="utf-8"))
                check_true(f"no model for {prompt!r}", written is not None)
                check_true(f"no null on disk after {prompt!r}",
                           all(written[j] is not None for j in mind.ALL_JOINTS))
            # and a movement command is still handled without the model too
            with contextlib.redirect_stdout(io.StringIO()):
                mind.ask_bot("turn left")
            check_true("a movement command still writes a full pose",
                       all(json.loads(mind.COMMAND_FILE.read_text(encoding="utf-8"))[j] is not None
                           for j in mind.ALL_JOINTS))
        finally:
            mind.STATE_FILE, mind.COMMAND_FILE = original_state, original_command
            mind._load_model, mind._ask_model = original_load, original_ask


# Phrasings written after the control words were frozen, and deliberately kept
# out of both datasets. They exist to catch rules that only ever agreed with the
# holdout they were chosen on.
NOVEL_CONTROL = (
    ("take me home", "home"),
    ("reset everything", "home"),
    ("reset the arm", "home"),
    ("go into standby", "home"),
    ("put it back to the default position", "home"),
    ("return to the neutral position", "home"),
    ("go back to the beginning", "home"),
    ("home", "home"),
    ("status?", "status"),
    ("how's the arm doing", "status"),
    ("where is the arm", "status"),
    ("what is your current position", "status"),
    ("is the arm moving", "status"),
    ("are you ready", "status"),
    ("help", "help"),
    ("what do you understand", "help"),
    ("list your commands", "help"),
)
NOVEL_NOT_CONTROL = (
    "don't go home yet",
    "never go home",
    "no need to go home",
    "park the arm",
    "are you still there",
    "helpful tilt up",
    "tell me a joke",
    "what is the weather like",
    "who are you",
    "someone told me ten things",
    "turn left",
    "tilt up",
    "move up-left",
    "open the gripper",
    "set joint two to 60",
    "move the base to 135",
    "hold on",
    "make a fist",
)


def test_novel_control_phrasings():
    for prompt, action in NOVEL_CONTROL:
        command = mind.build_local_command(prompt)
        check(f"novel phrasing resolves: {prompt!r}",
              None if command is None else command["action"], action)
    for prompt in NOVEL_NOT_CONTROL:
        command = mind.build_local_command(prompt)
        if command is not None:
            check_true(f"novel phrasing is not stolen by a control word: {prompt!r}",
                       command["action"] not in ("home", "status", "help"))


def main():
    for name, function in sorted(globals().items()):
        if name.startswith("test_") and callable(function):
            function()
    print(f"{CHECKS} checks run, {len(FAILURES)} failed")
    for failure in FAILURES:
        print(f"  FAIL {failure}")
    return 1 if FAILURES else 0


if __name__ == "__main__":
    raise SystemExit(main())
