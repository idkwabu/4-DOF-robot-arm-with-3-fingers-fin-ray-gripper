# DUM-E

A 4-DOF robot arm with a 3-finger gripper, driven by natural-language commands and a
locally-run fine-tuned language model. No GPU required — the base model is small enough
to run on CPU, although `device_map="auto"` will use a GPU if one happens to be present.

DUM-E parses plain English ("tilt the arm up", "turn all the way left", "open the
gripper"), resolves it against a fixed joint vocabulary, and writes a resolved pose to
`mind/robot_command.json` for whatever consumes it. A deterministic parser handles the
common cases instantly; a fine-tuned [Qwen3-0.6B](https://huggingface.co/Qwen/Qwen3-0.6B)
handles the phrasing the parser does not recognise.

---

## Table of contents

- [Status and limitations](#status-and-limitations)
- [Quickstart](#quickstart)
- [Architecture](#architecture)
- [The language pipeline](#the-language-pipeline)
- [Direction and angle reference](#direction-and-angle-reference)
- [Kinematics at a glance](#kinematics-at-a-glance)
- [Mathematics](#mathematics)
- [Training the model](#training-the-model)
- [File formats](#file-formats)
- [Verification](#verification)
- [Known issues](#known-issues)
- [Further documentation](#further-documentation)

---

## Status and limitations

Read this section before assuming any of the above works on hardware.

| | Status |
|---|---|
| Natural-language parsing | Working, 91.0% of the holdout set |
| Joint resolution and validation | Working |
| Forward / inverse kinematics | Working as a library, broken via the `RobotArm` facade (see [known issues](#known-issues)) |
| Tkinter GUIs | Working |
| Fine-tuned model | Working, 99.9% end-to-end — **shipped weights predate the current dataset** |
| **Hardware / servos** | **Not implemented. There is no serial, no GPIO, no firmware.** |
| **"Fin-ray" gripper** | **Kinematic only.** Three rigid straight fingers. No tendons, no underactuation, no compliant elements. |
| Hand / gesture recognition | Not implemented. `features_macro/hand_recognition.py` is a 0-byte placeholder. |

The repository name says "fin-ray" because that is the intended end state. What exists
today is a rigid three-finger cone gripper. `movements/robot_arm.py` states this in its
own module docstring, and nothing in the code contradicts it.

Nothing in this repository talks to a physical robot. The output of a command is a JSON
file naming the action, the direction, and five absolute joint angles. What you do with
it is up to you.

---

## Quickstart

Requires **Python 3.13** (developed against 3.13.15).

```bash
git clone https://github.com/idkwabu/4-DOF-robot-arm-with-3-fingers-fin-ray-gripper.git
cd 4-DOF-robot-arm-with-3-fingers-fin-ray-gripper

python -m venv .venv
# Windows:   .venv\Scripts\activate
# macOS/Linux: source .venv/bin/activate

pip install torch transformers matplotlib numpy
```

### Entry points

| Command | What it is |
|---|---|
| `python Dum-E.py` | Main GUI: text box, live 3D preview, parser and model together |
| `python mind/mind.py` | The brain on its own, as a CLI. Useful for debugging the pipeline |
| `python movements/robot_arm_gui.py` | Kinematics sandbox: sliders, FK/IK, reach envelope |

`Dum-E.py` and `mind/mind.py` both need the model weights to answer anything the
deterministic parser does not cover. `movements/robot_arm_gui.py` does not.

### Model weights are not in the repository

The adapter (10 MB) is committed. The merged model (1.19 GB) is **gitignored** — it is
too large for GitHub, and rebuilding it from the adapter is deterministic. If you have
just cloned:

```bash
python mind/merge_robot.py
```

`merge_robot.py` loads the base model from Hugging Face in fp16 on CPU, applies the
adapter, and writes 2 GB shards, so **it needs a network connection and a few GB of disk**.
`movements/robot_arm_gui.py` needs no model and works immediately.

If the weights are missing, `Dum-E.py` and `mind/mind.py` do **not** degrade gracefully.
`_load_model()` has no error handling, so the first prompt that needs the model raises an
unhandled exception from `from_pretrained(..., local_files_only=True)` and the process
exits. You will get a stack trace, not a message telling you to run `merge_robot.py`.

### Running the tests

```bash
python mind/test_resolver.py
```

---

## Architecture

Three subsystems. They share one angle contract and are otherwise independent.

```
   natural language
          |
   +------+------+
   |             |
Dum-E.py     mind/mind.py          <- language layer
(parser +    (same parser,
 Tkinter      same resolver,
 + 3D view)    no GUI)
   |             |
   +------+------+
          |
   joint angles (base, joint1, joint2, joint3, gripper)
          |
   mind/robot_command.json         <- the contract
          |
   movements/robot_arm.py          <- kinematics layer
   (FK / IK / reach / gripper)
```

### Which file owns what

| File | Responsibility |
|---|---|
| `Dum-E.py` | Main GUI. Contains its own copy of the parser and its own copy of the kinematics. |
| `mind/mind.py` | The language layer alone, as a CLI. Reference copy of the parser; the two copies are held byte-identical in behaviour by the differential harness. |
| `mind/system_prompt.txt` | The contract handed to the model. Defines actions and directions. |
| `mind/generate_robot_dataset.py` | Turns seed phrases into the training set. |
| `mind/train_robot.py` | QLoRA fine-tuning. |
| `mind/merge_robot.py` | Merges the adapter into a standalone model. |
| `mind/verify_model.py` | End-to-end accuracy and prompt-staleness checks. |
| `mind/test_resolver.py` | Resolver regression suite. |
| `mind/robot_dataset.jsonl` | 3990 training rows. |
| `mind/robot_dataset_eval.jsonl` | 1716 holdout rows. |
| `movements/robot_arm.py` | Forward/inverse kinematics, gripper, reach envelope. |
| `movements/robot_arm_gui.py` | Tkinter + Matplotlib control panel for the above. |
| `features_macro/hand_recognition.py` | Empty. Placeholder for a future gesture layer. |

### Why the duplication

`Dum-E.py` and `mind/mind.py` each contain the same parser and the same resolver. `Dum-E.py`
also contains its own copy of the forward kinematics. This is deliberate: it keeps the GUI a
single file that runs standalone, and it guarantees the GUI and the CLI can never drift on
the angle contract.

The cost is real and is documented under [known issues](#known-issues) — the two kinematic
copies have **different joint limits**, so they are not interchangeable.

---

## The language pipeline

Implemented in both `Dum-E.py` and `mind/mind.py`, in identical form.

```
input text
    |
    v
[1] build_local_command()   deterministic parser
    |                       -> action + direction, or None
    |
    | None  (parser abstains)
    v
[2] model.generate()        Qwen3-0.6B, fine-tuned
    |                       -> action + direction
    v
[3] resolve_command()       validate, clamp, apply to state
    |
    v
mind/robot_command.json     action, direction, five absolute angles
```

### Parser precedence

`build_local_command()` returns a decision on the first rule that matches, and the order
is not the order you would guess. Control words come **last** on purpose.

1. **Stop** — `STOP_PHRASES`, a 14-phrase frozenset. Checked first, so "stop" wins even
   inside a longer sentence. `STOP_NEGATED` un-matches "don't stop" / "no need to stop".
2. **Bare hard turn** — "all the way left" and "hard left" name no turn verb, so without
   this they would fall through to the model, which only knows the soft labels and would
   answer with a 45° turn.
3. **Bare middle** — "middle", "center", "straight ahead" name a turn without naming a
   turn verb. Without this they would be detected and then never dispatched. Resets the
   base to 90° and leaves every other joint untouched.
4. **Direction plus verb** — turn, tilt, or move, with an action verb, a bare hard turn, or
   a bare middle. This is the main path.
5. **Middle conflict** — a middle word beside a real left or right. Returns `None` and
   defers to the model, because the base-angle reading and the arm-pitch reading are
   genuinely different and the English is ambiguous.
6. **Overrides with no direction** — gripper open/close, and "set joint 2 to 45". These
   are reached only when no direction was detected.
7. **Control words** — home, status, help. Tried **last**, so a prompt that also carries a
   real direction or a number still moves: "help me turn left" is a turn, not a help
   request. Reaching this point means nothing moved, so a bare control word is the whole
   intent and the model is not needed to hear it.
8. **`None`** — abstain, and let the model answer.

Within step 7 the order is `home` → `status` → `help`.

### Turn magnitude

A plain turn is **45° off middle**. Guard words promote it to the full 90° stop:

- `HARD_NEAR` — "fully", "completely", "entirely", "hard", "far", "max" immediately
  preceding the direction, optionally with "to the" / "towards the".
- `HARD_ANYWHERE` — "all the way" or "as far as" anywhere in the sentence.

So `"turn left"` → 135°, and `"turn all the way left"` → 180°.

### Compound directions

Six of the thirteen presets combine a base angle with arm angles — the four diagonals
plus `up-middle` and `down-middle`. `up-left` and `down-left` are mirrored pairs at base
135; `up-right` and `down-right` are mirrored at base 45; `middle`, `up-middle` and
`down-middle` all sit at base 90 and differ only in arm pitch. The remaining seven set
either the base alone or the arm alone.

### Model fallback

When the parser abstains, the model runs. The model's output is parsed and validated with
the same resolver; a malformed or unknown `(action, direction)` pair is rejected. If the
model output is also unusable, the resolver falls back to the neutral state rather than
writing a partial command.

### The one deliberate abstention

The abstention at step 5 is the only place the parser knowingly declines an answer it
could guess at: "turn left to the middle" and "go to the middle on your left" both name a
base target and an arm target, and the English genuinely admits both readings.

Bare `middle` with no direction at all is *not* an abstention; it is handled directly by
rule 3, and never reaches the model.

---

## Direction and angle reference

All values in degrees.

### The thirteen presets

```python
DIRECTION_PRESETS = {
    "up":           {"joint1": 120, "joint2": -60, "joint3": -30},
    "down":         {"joint1":  60, "joint2": -90, "joint3": -60},
    "left":         {"base": 135},
    "right":        {"base":  45},
    "hard-left":    {"base": 180},
    "hard-right":   {"base":   0},
    "up-left":      {"base": 135, "joint1": 120, "joint2": -60, "joint3": -30},
    "up-right":     {"base":  45, "joint1": 120, "joint2": -60, "joint3": -30},
    "down-left":    {"base": 135, "joint1":  60, "joint2": -90, "joint3": -60},
    "down-right":   {"base":  45, "joint1":  60, "joint2": -90, "joint3": -60},
    "middle":       {"base":  90},
    "up-middle":    {"base":  90, "joint1": 120, "joint2": -60, "joint3": -30},
    "down-middle":  {"base":  90, "joint1":  60, "joint2": -90, "joint3": -60},
}
```

| Direction | Action | base | joint1 | joint2 | joint3 |
|---|---|---|---|---|---|
| `up` | tilt | — | 120 | -60 | -30 |
| `down` | tilt | — | 60 | -90 | -60 |
| `left` | turn | 135 | — | — | — |
| `right` | turn | 45 | — | — | — |
| `hard-left` | turn | 180 | — | — | — |
| `hard-right` | turn | 0 | — | — | — |
| `up-left` | move | 135 | 120 | -60 | -30 |
| `up-right` | move | 45 | 120 | -60 | -30 |
| `down-left` | move | 135 | 60 | -90 | -60 |
| `down-right` | move | 45 | 60 | -90 | -60 |
| `middle` | turn | 90 | — | — | — |
| `up-middle` | move | 90 | 120 | -60 | -30 |
| `down-middle` | move | 90 | 60 | -90 | -60 |

Omitted columns are left at their current value. This is what makes `middle` non-destructive.

### Parser presets versus model vocabulary

The model sees **12** directions; the parser has **13** presets. The extra two are
`hard-left` and `hard-right`, which are parser-internal: the model only ever emits `left`
or `right`, and the parser upgrades them to the hard variant when a guard word is present.
The model also has a `default` direction for non-spatial actions (`home`, `stop`, `status`,
`help`), giving 14 values in `ALL_DIRECTION_VALUES` in total.

### Joint limits and defaults

```python
JOINT_RANGES = {
    "base":   (0, 180),      # 90 is mechanical centre, 0 and 180 are the stops
    "joint1": (0, 180),
    "joint2": (-150, 150),
    "joint3": (-150, 150),
    "gripper": (0, 100),
}

DEFAULT_STATE = {"base": 90, "joint1": 90, "joint2": -60, "joint3": -60, "gripper": 50}
MIDDLE_BASE = 90
```

`MIDDLE_BASE` is the reason bare `middle` is a single base reset: the arm's neutral facing
is 90°, not 0°.

### Actions

Seven: `turn`, `tilt`, `move`, `home`, `stop`, `status`, `help`. `turn` is base-only,
`tilt` is arm-pitch-only, `move` is both. The split is enforced by `is_valid_command()`, so
a `tilt` carrying a base angle is rejected rather than silently half-applied.

---

## Kinematics at a glance

The full derivation, the signed-radius reasoning, and the effective-forearm treatment live
in [`movements/README.md`](movements/README.md), which is re-derived from the source and
agrees with it. Where anything still conflicts, the source is correct.

### Link geometry

| Symbol | Value | Meaning |
|---|---|---|
| `L0` | 50 mm | Fixed vertical base column, not a controlled joint |
| `L1` | 300 mm | Upper arm, driven by `joint1` |
| `L2` | 250 mm | Forearm, driven by `joint2` |
| `L3` | 70 mm | Wrist link, driven by `joint3` |
| `WRIST_STEM_ANGLE` | -60° | Fixed wrist-to-finger offset |
| `GRIPPER_FINGERS` | 3 | |
| `FINGER_LENGTH` | 50 mm | |
| `FINGER_OPENING_DEG` | 30° | Half-angle of the finger cone |
| `POSITION_TOLERANCE_MM` | 0.1 mm | FK/IK agreement tolerance |

### Four joints, three arm links

`L0` is fixed. The four controlled angles are `theta_base`, `joint1`, `joint2`, `joint3`.
The three movable arm links are `L1`, `L2`, `L3`. Some older notes describe this arm as
"3 DOF" because they predate `joint3`; the code and the `JointAngles` dataclass both have
four fields.

### `forward_kinematics` returns the grip point

Not the wrist. The return value is the point between the fingertips, which is why the
`L3` offset and the finger spread are folded into the result. A sanity check:

```python
import robot_arm as ra

ra.forward_kinematics(45, 30, -20, -60, ra.DEFAULT_CONFIG, 1.0)
# -> CartesianPoint(x=409.3004, y=409.3004, z=156.6182) mm
```

### Reach envelope

Measured from the `joint1` pivot at `(0, 0, L0)`, **not** from the world origin. The
envelope is **not a single number** — it depends on `joint3`, because the effective
forearm `m` is measured from `L2` along a direction set by `joint3`:

| `joint3` | Open (`opening=1`) | Closed (`opening=0`) |
|---|---|---|
| −150° | 137.90 – 462.10 mm | 142.08 – 457.92 mm |
| −120° | 83.18 – 516.82 mm | 83.44 – 516.56 mm |
| −90° | 25.52 – 574.48 mm | 22.69 – 577.31 mm |
| −60° (default) | 21.97 – 621.97 mm | 26.96 – 626.96 mm |
| −30° | 52.70 – 652.70 mm | 58.97 – 658.97 mm |
| **0°** | 63.30 – **663.30 mm** | 70.00 – **670.00 mm** |
| +60° | 21.97 – 621.97 mm | 26.96 – 626.96 mm |
| +90° | 25.52 – 574.48 mm | 22.69 – 577.31 mm |
| +150° | 137.90 – 462.10 mm | 142.08 – 457.92 mm |

**Maximum reach is 663.30 mm open / 670.00 mm closed, at `joint3 = 0°`** — not at the
`joint3 = -60°` default, which is the figure most references quote.

The inner bound is a signed quantity reported as a distance. `m` crosses `L1 = 300 mm`
at `joint3 = ±75°`, where the reported inner bound collapses to 0.00 mm; on either side
of that the arm can reach essentially its own base. See
[known issue 14](#14-absl1---m-hides-a-sign-change).

Closing the gripper *raises both bounds*: the on-axis grip depth `l_g` grows from
43.30 mm to 50.00 mm as the fingers converge, which lengthens the effective forearm.

### Verification

`verify_fk_ik()` round-trips 40 random poses through FK and back:

```
Position error: 0.000000 mm  (tolerance 0.1 mm)
Max angle round-trip error: 0.000000 deg
Samples: 40/40 passed
```

`inverse_kinematics()` also supports `joint3="auto"`, which sweeps `joint3` over its full
range to find the configuration that reaches furthest from the base origin.

---

## Mathematics

All angles are stored in radians internally; every public function takes and returns
degrees. Lengths are millimetres. This section documents what `movements/robot_arm.py`
actually computes — each equation is paired with its verified value.

### Grip geometry

The three fingers sit on a cone about the tool axis. `opening` scales their spread:

```
spread = FINGER_OPENING_DEG · opening            spread ∈ [0°, 30°]
l_g    = FINGER_LENGTH · cos(spread)             on-axis grip depth [mm]
```

`l_g` is the projection of a finger onto the tool axis. Because it is a **cosine**, the
depth is *shortest* when the fingers are most open:

| `opening` | `spread` | `cos(spread)` | `l_g` |
|---|---|---|---|
| 1.00 (open) | 30.0° | 0.866025 | 43.301 mm |
| 0.75 | 22.5° | 0.923880 | 46.194 mm |
| 0.50 | 15.0° | 0.965926 | 48.296 mm |
| 0.25 | 7.5° | 0.991445 | 49.572 mm |
| 0.00 (closed) | 0.0° | 1.000000 | 50.000 mm |

At `opening = 0` the fingers converge **on** the tool axis at depth `FINGER_LENGTH`, so
the three tips and the grip point coincide.

### The effective forearm

This is the central trick, and the reason the arm is tractable.

`L2`, `L3` and the grip depth `l_g` are rigidly fixed relative to each other, so their
*vector sum* can be collapsed into a single magnitude and offset angle. Treating the
plane as complex numbers:

```
z  ≡ L2 + (L3 + l_g) · e^(iφ),   φ = WRIST_STEM_ANGLE = -60°
m  = |z|            effective forearm length
δ  = arg(z)         effective forearm offset angle
```

`m` is the distance from `joint2` to the grip point when the forearm is straight; `δ` is
how far the folded tool axis is rotated away from `L2`. Together they turn a **three-link
arm into a plain two-link arm** with links `(L1, m)`.

For the stem alone, with no finger contribution:

```
m = |250 + 70·e^(i(-60°))| = 291.376 mm      δ = -12.008°
```

Compare that with the naive `L2 + L3 = 320 mm`. Folding saves **28.624 mm**, because `L3`
is pitched −60° rather than collinear with `L2`. Treating the links as a simple sum
overstates the arm's length by nearly a tenth.

### Forward kinematics

FK never builds a rotation matrix. It accumulates in a **radial–height plane**, then
rotates the whole result by the base angle at the end — which is why the base joint
cannot tilt the arm, only spin it.

With `r` the horizontal distance from the Z axis and `z` the height:

```
joint1     : r = 0                       z = L0
link 2     : r₂ = L1 · cos(joint1)
             z₂ = L0 + L1 · sin(joint1)
link 3     : r₃ = r₂ + L2 · cos(joint1 + joint2)
             z₃ = z₂ + L2 · sin(joint1 + joint2)
link 4     : r_m = r₃ + L3 · cos(joint1 + joint2 + joint3)
             z_m = z₃ + L3 · sin(joint1 + joint2 + joint3)
base spin  : x = r_m · cos(θ_base)      y = r_m · sin(θ_base)
```

Note that each link angle is the **sum** of every joint above it — absolute link
orientations, not relative deltas. The grip point then advances a further `l_g` along the
tool axis, whose unit vector is
`ax = (cos(γ)cos(θ_base), cos(γ)sin(θ_base), sin(γ))` with `γ = joint1 + joint2 + joint3`.

Worked example, the `up` preset `(90, 120, -60, -30)`:

```
r₂ = 300·cos(120°)              = -150.000   z₂ = 50 + 300·sin(120°)   = 309.808
r₃ = -150 + 250·cos(60°)        =  -25.000   z₃ = 309.808 + 250·sin(60°)= 526.314
r_m = -25 + 70·cos(30°)         =   35.622   z_m = 526.314 + 70·sin(30°)= 561.314
x   = 35.622·cos(90°)           =    0.000   y   = 35.622·sin(90°)      =  35.622
```

The grip point then advances `l_g = 43.301` mm along the tool axis, whose unit vector at
`γ = 30°` is `(0, 0.866025, 0.500000)`, giving `(0.00, 73.12, 582.96)` mm. Verify it
yourself:

```python
import robot_arm as ra

ra.forward_kinematics(90, 120, -60, -30, ra.DEFAULT_CONFIG, 1.0)
# -> CartesianPoint(x=0.0, y=73.1218, z=582.9646) mm
```

### The distance identity

Because of the effective forearm, the grip point obeys a two-link law of cosines. Let
`d` be the distance from the `joint1` pivot to the target — **measured from the pivot, not
the world origin**:

```
d² = L1² + m² + 2 · L1 · m · cos(joint2 + δ)
```

This single equation is what makes IK a short closed form rather than a numerical search.
It has been verified across 2880 poses with a maximum error of **2.3 × 10⁻¹³ mm**.

It also yields the reach bounds, `d ∈ [|L1 - m|, L1 + m]`, and it explains why `joint2`
depends only on `d` — a consequence shared by every target at the same distance.

### Inverse kinematics

Five steps, no iteration:

```
1.  θ_base = atan2(y, x)                       the base isolates the azimuth
2.  r = √(x² + y²)                             collapse to the radial–height plane
    height = z - L0
    d = √(r² + height²)
3.  cos(te′) = (d² - L1² - m²) / (2 · L1 · m)  law of cosines, te′ = joint2 + δ
    te_abs = acos(clamp(cos(te′), -1, 1))
4.  ψ = atan2(height, r)                       bearing of the target
    γ = acos(clamp((L1² + d² - m²) / (2 · L1 · d), -1, 1))
5.  joint1 = ψ ± γ                             ± chosen by joint2_up
    joint2 = ±te_abs - δ                       the δ is removed to get the physical angle
```

The signs are forced, not free: `joint2_up=True` gives `joint1 = ψ + γ` and
`te′ = -te_abs`, while `joint2_up=False` gives `joint1 = ψ - γ` and `te′ = +te_abs`.
Either way `joint2 = te′ - δ`, because `δ` was folded into `m` on the way in and must come
back out.

Targets outside `[|L1 - m|, L1 + m]`, or within `MIN_D_SHARE_LIMIT` of the pivot, raise
`ValueError`. Joint limits are deliberately **not** applied here — use
`check_joint_limits()` separately. Geometric reachability and joint limits are two
different questions; see [issue 2](#2-the-two-kinematic-copies-have-different-joint-limits).

```python
import robot_arm as ra

a = ra.inverse_kinematics(300, 0, 200, joint2_up=True, opening=1.0)
# JointAngles(theta_base=0.0, joint1=87.15, joint2=-97.10, joint3=-60.0)
ra.forward_kinematics(a.theta_base, a.joint1, a.joint2, a.joint3, ra.DEFAULT_CONFIG, 1.0)
# -> (300.00, 0.00, 200.00) mm, exactly on target
```

### Automatic `joint3` selection

Because `m` depends on `joint3`, a target that is unreachable at one wrist angle may be
reachable at another. `joint3="auto"` sweeps the full `joint3` range in 5° steps,
preferring values nearest the −60° default, and returns the first configuration that is
both geometrically reachable and inside every joint limit:

```python
import robot_arm as ra

ra.inverse_kinematics(600, 0, 0, joint3="auto", opening=1.0)
# -> JointAngles(theta_base=0.0, joint1=10.298, joint2=-11.331, joint3=-60.0)
```

Near targets `(50, 0, 50)` reach it with `joint3 = -75°`, the folded configuration, which
is why the inner reach bound collapses to nearly zero there.

---

## Training the model

### The pipeline

```
system_prompt.txt  +  seed phrases
        |
        v
generate_robot_dataset.py      expand, validate, balance, split
        |
        +--> robot_dataset.jsonl         3990 rows   (training)
        +--> robot_dataset_eval.jsonl    1716 rows   (holdout, 10%)
        |
        v
train_robot.py                 QLoRA fine-tune
        |
        v
dum-e-qwen3-robot/             adapter, 10,143,888 bytes
        |
        v
merge_robot.py                 fp16 CPU merge, 2 GB shards
        |
        v
dum-e-qwen3-merged/            standalone model, 1,192,134,784 bytes
```

### Dataset construction

124 seed phrases, grouped by action — `turn` 16, `tilt` 14, `move` 31, `home` 12,
`stop` 11, `status` 16, `help` 24 — expand into 15 `(action, direction)` classes. Each
seed is decorated with 12 prefixes and 10 suffixes, giving up to 120 candidate rows, which
are then deduplicated with `sorted(set(rows))` so one seed cannot contribute the same
prompt twice. A per-class counter caps output at `ROWS_PER_CLASS = 570`, and a 10% slice
(`EVAL_FRACTION`) is held back.

Note that the dedup is *within* a seed. There is no cross-seed collision check, so two
different seeds that happen to produce the same normalised prompt with different targets
would both survive.

| Action | Direction | Train rows |
|---|---|---|
| `move` | `up-left` | 125 |
| `move` | `up-right` | 133 |
| `move` | `down-left` | 80 |
| `move` | `down-right` | 69 |
| `move` | `up-middle` | 74 |
| `move` | `down-middle` | 89 |
| `tilt` | `up` | 291 |
| `tilt` | `down` | 279 |
| `turn` | `left` | 127 |
| `turn` | `right` | 174 |
| `turn` | `middle` | 269 |
| `home` | `default` | 570 |
| `stop` | `default` | 570 |
| `status` | `default` | 570 |
| `help` | `default` | 570 |
| | **Total** | **3990** |

The `default`-direction classes are saturated at the cap because their seeds expand
cleanly. The `move` compounds are under-represented, which is a direct cause of the
retrain regression documented below.

### Rebuilding from scratch

The system prompt is embedded verbatim in **every** training row. Changing
`system_prompt.txt` without regenerating the dataset leaves the two inconsistent, and
`verify_model.py` will flag the staleness.

```bash
python mind/generate_robot_dataset.py     # rewrite both JSONL files
python mind/train_robot.py                # QLoRA, ~31 min/epoch on CPU
python mind/merge_robot.py                # adapter -> standalone model
python mind/verify_model.py               # accuracy + staleness
```

### Training configuration

| Setting | Value |
|---|---|
| Base model | `Qwen/Qwen3-0.6B` |
| Training quantization | 4-bit NF4, double quant, bf16 compute |
| LoRA | `r=8`, `alpha=16`, `dropout=0.05` |
| Target modules | `q_proj`, `k_proj`, `v_proj`, `o_proj`, `gate_proj`, `up_proj`, `down_proj` |
| Steps | 1000, checkpoint every 250 |
| Batch | 1, gradient accumulation 4 (effective 4) |
| Optimiser | `paged_adamw_8bit`, lr `1e-4`, cosine, 100 warmup steps |
| Seed | 42 |

Inference does **not** use the training quantization. `_load_model()` loads the merged
model with `load_in_8bit=True` and bf16 compute, so the adapter is trained at 4-bit NF4
and served at 8-bit. That asymmetry is deliberate and is why accuracy must be measured
through `verify_model.py` rather than assumed from the training run.

### Two invariants to preserve when modifying training code

`train_robot.py` carries both of these in its module docstring, and both exist because
something went wrong without them.

- **`apply_chat_template(..., enable_thinking=False)`, exactly as at inference.** Qwen3
  emits a `<think>` preamble by default, so the model never reaches the JSON and every
  prediction is corrupted. Training and inference must agree on this.
- **`max_length` must exceed the longest formatted row.** It is now auto-computed as
  `max(lengths) + 8`. Do not hardcode it: the previous run used 256 against roughly
  380-token rows, which silently truncated the answer off the end of every example while
  training loss still looked healthy.

---

## File formats

### `mind/robot_command.json`

The output contract, written by `save_command()`. Seven keys, always all seven, never a
`null` — the file records what was done and where the arm ended up:

```json
{
  "action": "home",
  "direction": "default",
  "base": 90,
  "joint1": 90,
  "joint2": -60,
  "joint3": -30,
  "gripper": 50
}
```

A resolver command only names the joints it actually moves, so the rest arrive as `null`
and mean "unchanged". `absolute_command()` fills each of those from memory, so the emitted
pose is the **full post-command pose**: an angle the command names is the new one, an
angle it omits is the one the arm is already holding. `null` never reaches the disk.

### `mind/robot_state.json`

The remembered pose, per joint:

```json
{
  "base": 90,
  "joint1": 90,
  "joint2": -60,
  "joint3": -60,
  "gripper": 50
}
```

`load_state()` validates every value against `JOINT_RANGES` and falls back to that joint's
default alone, so one bad value never erases the whole remembered pose. A missing or
unparseable file falls back to `DEFAULT_STATE` outright.

Both files are written through `_write_json()`, which serialises to a sibling `.tmp` file
and then renames it into place. The rename is atomic, so a crash or a full disk cannot
leave a half-written state file behind.

---

## Verification

Measured on the committed state, not aspirational. Note that only some of these are
committed scripts — the rest were run ad hoc and are not in the repository.

| Check | Where it lives | Result |
|---|---|---|
| Resolver suite | `mind/test_resolver.py` | **736 checks, 0 failed** |
| FK -> IK -> FK round-trip | `robot_arm.verify_fk_ik()` | **40 / 40** at 0.000000 mm |
| Dataset prompt staleness | `mind/verify_model.py` | **0 stale rows** of 1716 |
| End-to-end pipeline | `mind/verify_model.py` | **1679 / 1681** = 99.9% |
| Model on its own share | `mind/verify_model.py` | **119 / 120** = 99.2% |
| Parser coverage of holdout | ad hoc | **91.0%** (1561 / 1716) answered locally |
| Differential parser harness | ad hoc | **57,651 / 57,651**, worst error 0.0000 mm |
| Golden geometry suite | ad hoc | **22 / 22** |
| Naming audit | ad hoc | **11 files, 0 problems** |
| GUI smoke test | ad hoc | **0 failures** |

The parser coverage number is the one that explains the rest. The deterministic parser
answers 91.0% of the holdout on its own, so the model only ever sees the remaining ~9% —
and on that slice it scores 99.2%. The end-to-end figure is the whole pipeline, parser
and model combined.

Reproduce the committed checks with:

```bash
python mind/test_resolver.py
python mind/verify_model.py
```

---

## Known issues

Everything below is a real, current defect. None of it is hypothetical.

### 1. The `RobotArm` facade is missing `joint3` and passes the wrong arity

`movements/robot_arm.py` defines `forward_kinematics` with six parameters. The
`RobotArm.forward_kinematics` method at **`robot_arm.py:1099`** declares only four —
`joint3` is missing from its signature entirely — and then calls through with five
arguments, so `self.config` lands in the `joint3` slot. The CLI's first menu option
crashes with `TypeError: must be real number, not RobotConfig`.

The same omission appears at **`robot_arm.py:1227`**, which calls
`forward_kinematics(a.theta_base, a.joint1, a.joint2, config, opening)` without
`a.joint3`. Line 1266 in the same file gets it right, so the two are inconsistent with
each other.

Five of the seven public `RobotArm` methods are affected. Only `plot` and `verify` work:

| Method | Status |
|---|---|
| `check_joint_limits` | broken — 3 required arguments missing |
| `forward_kinematics` | broken — 1 required argument missing |
| `gripper_pose` | broken — 3 required arguments missing |
| `inverse_kinematics` | broken — 3 required arguments missing |
| `is_reachable` | broken — 3 required arguments missing |
| `plot` | works |
| `verify` | works |

`RobotArm.inverse_kinematics` has the mirror problem: it accepts no `joint3` and no
`config`, so it cannot reproduce the module-level signature it shadows.

The module-level functions are all correct, which is why the GUI and the tests pass.

### 2. The two kinematic copies have different joint limits

| Joint | `mind` / `Dum-E.py` | `movements/robot_arm.py` |
|---|---|---|
| `base` | 0 – 180 | -180 – 180 |
| `joint1` | 0 – 180 | -90 – 110 |
| `POSITION_TOLERANCE_MM` | 0.5 | 0.1 |

The language layer cannot command a base angle the kinematics layer would consider valid.
Worse, `movements/robot_arm.py` is the more restrictive one, so the duplicate is not merely
redundant — it disagrees with the code that actually plans the motion.

### 3. The shipped adapter predates the current dataset

`mind/dum-e-qwen3-robot/` is the **pre-retrain** adapter. The committed datasets contain
`up-middle` and `down-middle`, which the shipped weights have never been trained on.
Behaviour for those two directions is handled by the deterministic parser, not the model.

### 4. A full retrain regressed and was rolled back

A 1000-step retrain on the current data (2026-09-28) reduced accuracy:

| | Shipped | Retrained |
|---|---|---|
| End-to-end | 99.9% | 96.2% |
| Model share | 119 / 120 | 57 / 120 |

Training itself looked healthy — loss 3.29 → 0.02, token accuracy 0.994 — and the
`middle` probe improved from 13/28 to 22/28. The regression is a `status` → `help` collapse
on prompts like "summarise the last command", which are labelled `status` but sit close to
`help` phrasing.

The retrain was rolled back and the original weights restored byte-for-byte. The regressed
artifacts are preserved at
`%TEMP%\opencode\model_new_trained\` for inspection. **Do not retrain until the seed
overlap is fixed.**

### 5. A seed's verb contradicts its label

The seed `"Tilt down and center the base"` is labelled `move` / `down-middle`, but
"tilt" is a `tilt` action. This teaches the model that action words are unreliable, and it
is a contributing cause of issue 4.

### 6. `HELP_TEXT` does not list the compound middle directions

`HELP_TEXT` in both `Dum-E.py` and `mind/mind.py` advertises `move up-left/up-right/down-left/down-right`
but omits `up-middle` and `down-middle`, which the parser fully supports. Ask the model for
help and it will under-report its own vocabulary.

### 7. `_load_model()` has no error handling

Both `Dum-E.py` and `mind/mind.py` call `_load_model()` at the top level with no
`try`/`except`. Because the merged model is gitignored, a fresh clone hits this on the
first prompt that the parser cannot answer, and the user gets a `from_pretrained` stack
trace instead of "run `mind/merge_robot.py` first".

### 8. `mind/Dum-E.spec` cannot produce a working build

The PyInstaller spec packages `mind/mind.py` — the CLI — rather than `Dum-E.py`, and
specifies `datas=[]`, so the 1.19 GB merged model is not bundled. A frozen build will
start and then fail on its first model load.

### 9. An orphan file at the repository root

`robot_command.json` exists at the repo root. The brain reads and writes
`mind/robot_command.json`. Nothing reads the root copy. It is a leftover from the pre-`mind/`
layout and is gitignored.

### 10. Sub-documentation was stale — corrected in this release

Both sub-documents previously carried numbers derived from a gripper model the code never
used: `l_g = finger_length/2 · cos(spread)`, a pre-tripod **half-length** model. Under it,
`m_g = 306.29 mm` and reach is `6.29 – 606.29 mm`. The code uses the full
`l_g = FINGER_LENGTH · cos(spread)` (43.30 mm open), giving `m_g = 321.97 mm` at
`joint3 = -60°` and a **joint3-dependent** envelope whose maximum is 663.30 mm at
`joint3 = 0°`. The document and the code were mathematically consistent with *different*
gripper models; that, not a typo, is why every downstream number was off.

Also stale and now fixed:

- `movements/README.md` glossed DOF as 3, built `JointAngles` with three arguments
  against a four-field dataclass, omitted `joint3` from most FK/IK signatures and all
  `RobotArm` examples, described `WRIST_STEM_ANGLE` as a *fixed* stem kink rather than a
  commandable fourth joint, and omitted `joint3` from the `JOINT_LIMITS` listing.
- `mind/README-DUM-E.txt` reported `test_resolver.py` as having 211 assertions where the
  suite runs 736.

Every numeric claim in `movements/README.md` is now re-derived from
`movements/robot_arm.py`, and every Python block in both documents executes as written.
Trust the source code over any document for numeric values.

### 11. `features_macro/hand_recognition.py` is empty

A 0-byte file. No MediaPipe, no OpenCV, no gesture recognition of any kind. The directory
is a placeholder.

### 12. `explain.txt` has been removed

It was tracked but described a different system: an ESP32/Arduino C++ firmware with
`JOINT_PINS`, 150 mm links, and a `MOVE_JOINT` text protocol. None of that exists in this
repository. It was the design ancestor of the current `{"action", "direction"}` flow, not
documentation of it, so the deletion is committed and the file is gitignored.

### 13. The reach envelope depends on `joint3`, and dead code hides it

There are **two** functions that compute reach bounds:

| Function | Line | Uses |
|---|---|---|
| `_reach_bounds` | 206 | `φ` pinned to `WRIST_STEM_ANGLE` — **dead code, never called** |
| `_reach_bounds_with_joint3` | 439 | `φ = joint3` — used by `is_reachable` at line 659 |

The live path is correct. The dead one is only accidentally right when
`joint3 == WRIST_STEM_ANGLE == -60°`, which is the default, so its wrong answer looks
plausible. An earlier revision of this README published `21.97 – 621.97 mm` as *the*
reach envelope, taken from the dead function's assumption. Maximum reach is in fact
**663.30 mm** open / **670.00 mm** closed at `joint3 = 0°`.

`_reach_bounds` should be deleted, or made `joint3`-aware, so the two cannot disagree
again.

### 14. `abs(L1 - m)` hides a sign change

The inner reach bound is reported as `abs(L1 - m)`, but `L1 - m` is genuinely **negative**
across most of the range: `m` exceeds `L1 = 300 mm` for any `joint3` between roughly
−75° and +75°, and is symmetric in the sign of `joint3` because `m` depends on
`cos(joint3)`.

At `joint3 = ±75°` exactly, `m = L1`, and the reported inner bound collapses to
**0.00 mm** — the arm can reach its own base. That is correct as a *distance*, and it is
the number a caller should compare against, but it hides the fact that the arm is fully
folded there. The signed value is the more informative quantity.

---

## Further documentation

| Document | Contents | Reliability |
|---|---|---|
| `mind/README-DUM-E.txt` | Full parser and resolver reference, every rule, every regex | Current — assertion count corrected |
| `movements/README.md` | Kinematics derivation, signed-radius reasoning, effective forearm | Corrected in this release — equations, examples and reach values all re-derived from source |
| `mind/system_prompt.txt` | The model's exact contract | Source of truth |

For anything the sub-documents disagree on, the source code wins.

---

## License

No license file is present. All rights reserved by default.
