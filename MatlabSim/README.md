# DUM-E MATLAB / Simscape simulations

This folder holds two standalone MATLAB/Simscape visualizations of the DUM-E arm:

| Folder | Robot version | Entry point | Gripper |
|---|---|---|---|
| [`dum_e_v2/`](dum_e_v2/) | `dum_e_v2_mk16` (CAD mark 16) | `follow_dume_v2.m` | 3-finger linkage driven by a **servo revolute joint** |
| [`dum_e_v3/`](dum_e_v3/) | `dum_e_v3_mk19` (CAD mark 19) | `follow_dume_v3.m` | 2-jaw linkage driven by a **prismatic slider** |

Both are **visual followers**, not controllers. They read the same root-level
`D:\Dum-E\robot_state.json` that the Python GUI writes, and animate the CAD
mechanism in Simscape Mechanics Explorer. Neither sends anything to hardware.

---

## Table of contents

- [Quick start](#quick-start)
- [What is in each folder](#what-is-in-each-folder)
- [End-to-end data flow](#end-to-end-data-flow)
- [The state-file contract](#the-state-file-contract)
- [What the MATLAB runner actually does](#what-the-matlab-runner-actually-does)
- [The live pose source (Level-2 S-function)](#the-live-pose-source-level-2-s-function)
- [Signal routing and the gripper filter](#signal-routing-and-the-gripper-filter)
- [v2 specifics](#v2-specifics)
- [v3 specifics](#v3-specifics)
- [Model and data files](#model-and-data-files)
- [Limitations and caveats](#limitations-and-caveats)

---

## Quick start

From MATLAB, make the relevant folder current and call its runner:

```matlab
cd('D:\Dum-E\MatlabSim\dum_e_v2')
follow_dume_v2
```

or

```matlab
cd('D:\Dum-E\MatlabSim\dum_e_v3')
follow_dume_v3
```

By default the runner:

1. Launches `D:\Dum-E\Dum-E.py` with the project virtual-environment Python
   (`D:\Dum-E\.venv\Scripts\python.exe`) using the Windows `start` command.
2. Waits up to 30 s for `robot_state.json` to exist.
3. Rebuilds/validates the clean generated `.slx`, copies it to a `*_live.slx`,
   configures and saves that live copy, opens it, and runs one continuous
   simulation until **Ctrl+C** in MATLAB.

If the Python GUI is already open, skip step 1:

```matlab
follow_dume_v2(false)
follow_dume_v3(false)
```

Optional positional arguments are (shown here for v2; v3 has the identical
signature with `follow_dume_v3`):

```matlab
follow_dume_v2(startGui, stopTime, moveDuration, stateFile)
```

- `startGui` — `true`/`false` (default `true`)
- `stopTime` — simulation stop time in seconds (default `1e6`, effectively forever)
- `moveDuration` — seconds used for each new pose transition (default `0.5`)
- `stateFile` — alternative JSON pose file (default `D:\Dum-E\robot_state.json`)

Example:

```matlab
follow_dume_v3(false, 120, 0.5, 'D:\Dum-E\test_pose.json')
```

To force a rebuild of the clean source model from the URDF:

```matlab
follow_dume_v2('rebuildSource')   % or follow_dume_v3('rebuildSource')
```

---

## What is in each folder

### `dum_e_v2/`

| File | Purpose |
|---|---|
| `follow_dume_v2.m` | The **only entry point**. Contains the follower, state-file parser/validation, pose mapping, S-function callbacks, model rebuild logic, and gripper closure construction. |
| `dum_e_v2_mk16.urdf` | Open-loop tree: 15 links, 14 joints. Y-up, metres; STL meshes declared with `scale="0.001 0.001 0.001"` (CAD is in mm). |
| `loop_constraints.json` | Three CAD gripper loop closures: rod→finger joint frames (translation + rotation matrix per link). |
| `cad_inertia_estimates.json` | Per-link mass, COM and inertia tensor estimated from CAD STLs; total ≈ **164.469 kg**. |
| `dum_e_simscape_generated.slx` | Clean source model imported from the URDF with gravity and the three closures added. Never edited by the runner. |
| `dum_e_simscape_live.slx` | Working copy. Re-created on every run, configured with the live pose source and filters, then saved/opened. |
| `dum_e_simscape_legacy.slx` | Archived earlier live variant. Contains extra `DumeLiveClock` and `DumeHoldPosition/Velocity/Acceleration` blocks. Not used by `follow_dume_v2.m`. |
| `dum_e_v2_generated.avi` | Rendered v2 demo: 1345×820, 30 fps, 301 frames (~10 s). |
| `README.md`, `README_SIMSCAPE.txt` | Shorter per-version/export notes (see caveats below). |
| `meshes/*.stl` | Visual and collision geometry for all 15 links. |

### `dum_e_v3/`

| File | Purpose |
|---|---|
| `follow_dume_v3.m` | The v3 equivalent of the v2 runner, adapted to the slider gripper and four closure loops. |
| `dum_e_v3_mk19.urdf` | Open-loop tree: 15 links, 14 joints (13 revolute/continuous + the `gripper_slide` prismatic joint). |
| `loop_constraints.json` | Four CAD gripper loop closures. |
| `cad_inertia_estimates.json` | Per-link estimates; total ≈ **164.431 kg**. |
| `dum_e_v3_generated.slx` | Clean source model. |
| `dum_e_v3_live.slx` | Configured working copy, re-created every run. |
| `README.md`, `README_SIMSCAPE.txt` | Shorter per-version/export notes. |
| `meshes/*.stl` | Visual/collision geometry for v3 links. |

There is no v3 AVI and no legacy model in `dum_e_v3/`.

---

## End-to-end data flow

```text
Dum-E.py (Python GUI)
        |
        | writes six validated values (degrees / 0..100)
        v
robot_state.json  <------------------- read at 50 Hz by the S-function
        |
        v
DumeLivePoseSource (Level-2 MATLAB S-function)
        |  outputs q, qdot, qddot  (6-element vectors)
        v
three Demux blocks -> 5 arm joints directly
                  \-> gripper channel -> 3 continuous Transfer Fcn filters
                                        -> Simulink-PS Converter -> gripper joint
        |
        v
Simscape Mechanics Explorer animation
```

The five arm joints and the gripper joint are all set to
`MotionActuationMode = 'InputMotion'` and
`TorqueActuationMode = 'ComputedTorque'`. That means the joint motion is imposed
by the Simulink signal and Simscape computes the torque needed to produce it.
The clean generated source models instead leave them in `ComputedMotion` /
`NoTorque` mode.

Each joint target is pushed through a `nesl_utility/Simulink-PS Converter` with
`FilteringAndDerivatives = 'provide'` and `UdotUserProvided = '2'`, so the
position, velocity and acceleration are supplied explicitly rather than
differentiated numerically inside Simscape.

---

## The state-file contract

Both runners read exactly six fields and validate that each is numeric, scalar,
finite and inside its expected range:

| JSON field | Valid range | CAD joint (v2) | CAD joint (v3) | Mapping applied |
|---|---|---|---|---|
| `base` | `-180..180` deg | `base_yaw_joint` | `base_yaw` | negated |
| `joint1` | `-90..90` deg | `shoulder_pitch_joint` | `shoulder_pitch` | minus 90 deg |
| `joint2` | `-150..150` deg | `elbow_pitch_joint` | `elbow_pitch` | unchanged |
| `joint3` | `-120..120` deg | `wrist_pitch_joint` | `wrist_pitch` | unchanged |
| `gripper_roll` | `-180..180` deg | `wrist_roll_joint` | `wrist_roll` | unchanged |
| `gripper` | `0..100` | `gripper_servo_pivot_joint` | `gripper_slide` | see below |

In code the first five are:

```matlab
armPoseDeg = values(1:5) .* [-1 1 1 1 1] + [0 -90 0 0 0];
pose = deg2rad(armPoseDeg);
```

The `-90` offset on `joint1` is why the runners override the shoulder joint
limits in the live model to `[-pi, 0]` rad, even though the URDF declares
`[-pi/2, pi/2]`.

Gripper mapping:

- **v2:** `gripper = 0..100` → servo `0..59.3 deg` → radians.
- **v3:** `gripper = 0..100` → slider `-0.009..0.010 m` (i.e. `-9..10 mm`).

The file is read with up to five attempts, 10 ms apart, to survive a transient
lock or partial write while the GUI is rewriting it.

---

## What the MATLAB runner actually does

Both `follow_dume_v2.m` and `follow_dume_v3.m` are single files with local
functions. Their structure is identical except for names and gripper units.

### 1. Argument dispatch

The top-level function handles two special calls before the normal run:

- a `Simulink.MSFcnRunTimeBlock` argument → `setupPoseSource(block)`; this is
  how Simulink calls the file as an S-function during simulation;
- the string `'rebuildSource'` → `rebuildSourceModelCommand()`;

anything else goes to `runFollower(...)`.

### 2. Clean source validation / rebuild

`isCleanSourceModel()` loads the checked-in generated `.slx` and rejects it if:

- it contains any block whose name starts with `Dume`, `dumeRadians_` or
  `dumeUnits_` (i.e. it is a live copy, not a clean source);
- expected gripper closure joints are missing;
- gravity is not `[0 -9.80665 0]`.

v3 additionally checks that each closure block's `ReferenceBlock` is
`sm_lib/Joints/Spherical Joint`; v2 only checks the block names.

If the source is missing or dirty, `rebuildSourceModel()` deletes it, runs:

```matlab
smimport(<urdf>, 'ModelName', <generated model name>)
```

sets constant Y-up gravity `[0 -9.80665 0] m/s^2`, calls
`addGripperClosures()` using `loop_constraints.json`, saves the `.slx`, and
closes it. The runner therefore never permanently modifies the clean source.

### 3. Live model setup

- If a loaded source or live model is still simulating, the runner errors out.
- The live copy is closed if loaded, deleted implicitly by `copyfile`, and
  loaded again from the freshly copied file.
- Gravity on the live copy is set to **`[0 0 0]`** — disabled. The passive gripper
  linkage can lose closure under gravity, so live pose tracking is done
  gravity-free. The clean source keeps Y-up gravity.
- Joint targeting, the S-function, Demuxes, filters and PS converters are added.
- Model settings: `StopTime` from the argument, `EnablePacing = 'on'`,
  `PacingRate = '1'` (real-time animation), `MaxStep = '0.01'`.
  The saved config uses a variable-step auto solver with `RelTol = 1e-3`.
- The configured live model is saved back to `*_live.slx`, opened, and started
  with `sim(model)`. An `onCleanup` callback stops the simulation if the
  function exits or errors.

---

## The live pose source (Level-2 S-function)

`DumeLivePoseSource` is a Level-2 MATLAB S-function whose function name is the
runner file itself (`follow_dume_v2` / `follow_dume_v3`). It is created with five
dialog parameters:

```text
'<stateFile>', 0.02, <moveDuration>, <gripperLow>, <gripperHigh>
```

- sample time `0.02 s` (50 Hz, hard-coded),
- move duration (default `0.5 s`),
- gripper endpoints (deg for v2, metres for v3).

It has **0 inputs and 3 outputs**, each a 6-element real double vector:
position, velocity, acceleration. Two Dworks of length `10*6 + 1 = 61` store
the current trajectory state (target pose, start pose/velocity/acceleration,
six quintic coefficients C0..C5, and start time).

Behaviour:

1. `Start` initializes target = current file pose, start = same pose, zero
   velocity/acceleration, and `StartTime = -duration`, so the first output is
   already the hold pose.
2. `Outputs` evaluates a quintic polynomial in normalized time `u = elapsed/duration`:

   ```text
   q(u)     = C0 + C1*u + C2*u^2 + C3*u^3 + C4*u^4 + C5*u^5
   qdot     = (C1 + 2*C2*u + 3*C3*u^2 + 4*C4*u^3 + 5*C5*u^4) / duration
   qddot    = (2*C2 + 6*C3*u + 12*C4*u^2 + 20*C5*u^3) / duration^2
   ```

   Coefficients are recomputed whenever a new target appears, preserving
   position, velocity **and** acceleration at the restart point and ending at
   the target with zero velocity and acceleration.
3. While a move is in progress the file is still polled. If the target changes
   mid-move, a new quintic starts from the *current* q/qdot/qddot.
4. Once a move is complete, `waitForTargetChange()` polls the file every
   `sampleTime` seconds and blocks until a new pose appears. This is why the
   model "waits at the current pose" between GUI changes.
5. `Update` copies the pending Dwork into the active one, keeping the S-function
   state consistent across Simulink's output/update phases.

---

## Signal routing and the gripper filter

Each of the three S-function outputs goes into its own Demux with six outputs.
Channels 1–5 (the arm joints) feed their `q`, `qdot`, `qddot` directly into the
corresponding Simulink-PS converter and revolute joint.

Channel 6 is different. A sampled-and-held gripper command made the closed CAD
linkage lose its constraints, so its position is passed through three continuous
Transfer Fcn blocks:

```text
tau   = max(0.01, moveDuration / 10)
den   = [tau^3  3*tau^2  3*tau  1]     % == (tau*s + 1)^3
num1  = [1]        -> filtered position
num2  = [1 0]      -> filtered velocity
num3  = [1 0 0]    -> filtered acceleration
```

The three filtered signals are wired into the three inputs of the gripper's PS
converter. This continuous path keeps the closed linkage solvable while the
five arm joints keep the much simpler direct path.

With the default `moveDuration = 0.5`, `tau = 0.05 s` and the denominator is
`[1.25e-4 7.5e-3 0.15 1]`. (The committed live snapshots were saved from a
short test with `moveDuration = 0.1`, hence `tau = 0.01` and a denominator of
`[1e-6 3e-4 0.03 1]`.)

---

## v2 specifics

The v2 gripper is a servo-driven, three-finger linkage. The URDF tree contains:

```text
l3_roll
 └─ gripper_servo_pivot_joint -> gripper_servo_horn
     └─ gripper_horn_crank_joint -> gripper_crank
         └─ gripper_crank_drive_joint -> gripper_drive_plate
             ├─ gripper_drive_rod_1_joint -> connecting_rod_1
             ├─ gripper_drive_rod_2_joint -> connecting_rod_2
             └─ gripper_drive_rod_3_joint -> connecting_rod_3

l3_roll
 ├─ gripper_finger_1_pivot_joint -> finger_1
 ├─ gripper_finger_2_pivot_joint -> finger_2
 └─ gripper_finger_3_pivot_joint -> finger_3
```

The three rod ends are *not* in the URDF tree. `addGripperClosures()` restores
them from `loop_constraints.json`:

- `cad_closure_rod_1_finger_1_joint`
- `cad_closure_rod_2_finger_3_joint`
- `cad_closure_rod_3_finger_2_joint`

For each loop it adds a `Rigid Transform` and a frame port inside both link
subsystems using the JSON translation/rotation, then adds a top-level
**Spherical Joint**. Although `loop_constraints.json` labels the type
`"revolute"`, spherical joints are used deliberately: a revolute joint adds a
redundant axis-alignment constraint that made the three-loop mechanism
singular whenever the servo moved.

Calibration notes from the v2 README:

- GUI `gripper = 0` → servo `0 deg` → fingers closed, matching the Python model.
- GUI `gripper = 100` → servo `59.3 deg` → about `30 deg` finger swing, matching
  `FINGER_OPENING_DEG` in `movements/robot_arm.py`.
- Do **not** drive the servo below `0 deg`; the linkage approaches a singularity
  near `-60 deg` where a finger flips and the solver can fail.

---

## v3 specifics

The v3 gripper replaces the servo/crank/plate/rod assembly with a symmetric
two-jaw slider mechanism:

```text
l3_roll
 ├─ gripper_slide (prismatic, -0.009..0.010 m) -> gripper_carriage
 │   ├─ gripper_pos_connector_hinge -> pos_connector
 │   └─ gripper_neg_connector_hinge -> neg_connector
 ├─ gripper_pos_lower_hinge  -> pos_lower  -> pos_jaw
 ├─ gripper_neg_lower_hinge  -> neg_lower  -> neg_jaw
 ├─ gripper_pos_upper_hinge  -> pos_upper
 └─ gripper_neg_upper_hinge  -> neg_upper
```

`loop_constraints.json` supplies four closure loops:

- `gripper_pos_connector_to_lower_hinge`
- `gripper_neg_connector_to_lower_hinge`
- `gripper_pos_upper_to_jaw_hinge`
- `gripper_neg_upper_to_jaw_hinge`

They are built the same way as v2 — JSON frame transforms plus top-level
**Spherical Joints** — using the same singularity-avoidance rationale. v3's
clean-source check is slightly stricter: it verifies the closure blocks really
are `sm_lib/Joints/Spherical Joint` references.

The slider limits come from Fusion (converted cm → m) and map directly from
GUI `gripper = 0..100` to `-9..10 mm`.

---

## Model and data files

### URDFs

Both URDFs are **open-loop trees**. They describe 15 links with STL visual and
collision meshes and per-link inertials (mass, COM origin, inertia tensor). The
loop closures are added only in MATLAB, from `loop_constraints.json`.

- Coordinate frame: **Y-up**.
- Units: metres; STL files are in millimetres and scaled by `0.001`.
- Joint effort/velocity values of `100` are schema placeholders, not measured
  actuator ratings.
- Several gripper joint limits/couplings are unverified.

### `loop_constraints.json`

Units are metres; rotations are link-to-joint matrices. Each loop gives two
frames (`frame_one`, `frame_two`) with:

- `link` — the link subsystem that receives the new frame port,
- `translation_m_in_link_frame`,
- `rotation_matrix_in_link_frame`.

The MATLAB builder creates a `Rigid Transform` from those values, exposes a new
frame connection, and joins the two frames with a spherical joint.

### `cad_inertia_estimates.json`

Schema version 1, with a `source_signature` identifying the CAD export. Method:
closed, consistently wound per-solid STL meshes integrated by tetrahedra using
the source CAD density (documented as **Steel, 7850 kg/m³**).

| | v2 | v3 |
|---|---|---|
| Links with estimates | 15 | 15 |
| Total estimated mass | 164.469 kg | 164.431 kg |
| Non-solid bodies omitted | 70 | 70 |
| Solids skipped as invalid | 6 | 9 |

Per link the file records `solid_count`, `used_count`, `non_solid_omitted`,
`skipped` reasons, `mass_kg`, `com_world_m`, `inertia_com_kg_m2` and
`min_inertia_eigenvalue_kg_m2`.

### `.slx` models

`.slx` files are ZIP containers of Simulink XML plus Simscape instrumentation
data. The generated sources contain the imported bodies/joints, the added
spherical closures and `MechanismConfiguration`. The live files add the
`Dume*` signal-generation blocks. The generated/live pairs are:

- v2: `dum_e_simscape_generated.slx` → `dum_e_simscape_live.slx`
- v3: `dum_e_v3_generated.slx` → `dum_e_v3_live.slx`

The runner copies source → live on every run, so the committed `*_live.slx`
files are simply the most recent saved snapshots, not canonical inputs. The v2
legacy file is an archived experiment that used explicit hold blocks
(`DumeHoldPosition`, `DumeHoldVelocity`, `DumeHoldAcceleration`,
`DumeLiveClock`) instead of the current continuous trajectory path.

---

## Limitations and caveats

1. **Preliminary dynamics.** Masses, COMs and inertias are estimates, not
   measured properties. Overlapping solids may be double-counted, STL
   tessellation is approximate, 70 non-solid bodies were omitted, and several
   solids were skipped for invalid inertia eigenvalues or non-manifold edges.
   Do not trust torque/dynamic results without replacing them with real data.

2. **Gravity is disabled in live mode.** The clean sources use
   `[0 -9.80665 0] m/s^2` in the Y-up frame, but the live copies set gravity to
   zero because the preliminary passive gripper linkage can lose closure under
   gravity. This is a visualization aid, not a dynamic study.

3. **The closed loops are spherical, not revolute.** `loop_constraints.json`
   says `"type": "revolute"`, but the builder intentionally creates spherical
   joints to remove a redundant constraint that caused solver failures.

4. **`README_SIMSCAPE.txt` is partly stale.** Both folders tell you to run a
   helper named `import_dume_simscape`; that function is **not present** in the
   repository. The supported workflow is `follow_dume_v2.m` /
   `follow_dume_v3.m`, which performs the import via `smimport` internally.

5. **Live `.slx` snapshots are not configuration files.** The committed live
   models were saved from short tests: they point at temporary state files in
   `%TEMP%` and use a 0.1 s move duration. Every real run overwrites them from
   the clean source with your arguments.

6. **Windows-only GUI launch.** `start "" /D ...` and
   `.venv\Scripts\python.exe` assume Windows. On another OS, pass `false` as
   the first argument and launch the Python GUI yourself.

7. **No hardware output.** The simulation only visualizes the pose read from
   JSON. Servo/Arduino control lives in the root `servo_controller.py` and is
   independent of these models.

8. **Gripper travel is deliberately restricted.** v2 must stay in `0..59.3 deg`
   (never below 0); v3 is limited to the CAD slider range `-9..10 mm`. The
   Python kinematics model uses a rigid curled-finger abstraction, whereas the
   Simscape models show the actual CAD linkage, so their gripper geometry is
   related but not identical.
