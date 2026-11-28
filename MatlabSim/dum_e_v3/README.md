# DUM-E v3 MATLAB/Simscape

The only MATLAB entry point for v3 is `follow_dume_v3.m`. It is the v3
equivalent of the optimized v2 runner: the live model is rebuilt from a clean
generated source each run, five arm joints keep the direct sampled command
path, and the gripper slider is driven through a continuous
position/velocity/acceleration filter so the closed linkage stays consistent.

Run the simulation from MATLAB:

```matlab
cd('D:\Dum-E\MatlabSim\dum_e_v3')
follow_dume_v3
```

If the Python GUI is already open:

```matlab
follow_dume_v3(false)
```

Optional arguments are `startGui`, `stopTime`, `moveDuration`, and `stateFile`;
for example:

```matlab
follow_dume_v3(false, 120, 0.5, 'D:\Dum-E\test_pose.json')
```

Each run saves the configured live model as `dum_e_v3_live.slx` in this folder.
The clean source `dum_e_v3_generated.slx` is rebuilt from
`dum_e_v3_mk19.urdf` automatically if missing or stale; force a rebuild with:

```matlab
follow_dume_v3('rebuildSource')
```

The GUI gripper opening (`gripper: 0..100`) maps to the v3 slider range
`-9..10 mm`, matching the URDF limits. The four CAD closure joints are
preserved as spherical joints, using the same singularity-avoidance approach
as v2. Live mode disables gravity because the preliminary passive linkage can
lose closure under gravity; the clean source retains Y-up gravity. Inertias are
preliminary estimates, and this simulation does not send commands to hardware.

---

## How the Python pose reaches a Simulink joint

The Python GUI does **not** talk to Simulink directly. It never names a block
and it never sets a block parameter. Its whole contribution is to write six
validated numbers to `D:\Dum-E\robot_state.json`. The MATLAB runner owns the
wiring: `follow_dume_v3.m` reads that file, converts the numbers to CAD units,
and `configureContinuousInputs()` decides which Simulink joint block receives
each value.

```text
Dum-E.py
   | writes base, joint1, joint2, joint3, gripper_roll, gripper
   v
robot_state.json
   | read by the DumeLivePoseSource S-function inside the .slx
   v
readDumePose()  -> radians + metres + joint mapping
   v
DumeLivePoseSource  -> q, qdot, qddot (6-element vectors)
   v
DumeSplitPosition / DumeSplitVelocity / DumeSplitAcceleration
   v
dumeUnits_<joint> Simulink-PS Converters
   v
revolute joints + gripper_slide prismatic joint
```

### Which JSON field drives which block

`readDumePose()` validates the six fields against
`[-180 180; -90 90; -150 150; -120 120; -180 180; 0 100]` and then applies:

```matlab
armPoseDeg = values(1:5) .* [-1 1 1 1 1] + [0 -90 0 0 0];
pose = [deg2rad(armPoseDeg), gripperSlideMeters].';
```

| `robot_state.json` field | Conversion | Joint block in the live model | Signal path |
|---|---|---|---|
| `base` | negated, then deg → rad | `base_yaw` | direct q/qd/qdd |
| `joint1` | `-90 deg`, then deg → rad | `shoulder_pitch` | direct q/qd/qdd |
| `joint2` | deg → rad | `elbow_pitch` | direct q/qd/qdd |
| `joint3` | deg → rad | `wrist_pitch` | direct q/qd/qdd |
| `gripper_roll` | deg → rad | `wrist_roll` | direct q/qd/qdd |
| `gripper` | `0..100` → `-0.009..0.010 m` | `gripper_slide` | **filtered** q/qd/qdd |

The `-90 deg` shift on `joint1` is why the runner overrides that joint's limits
in the live model to `[-pi, 0] rad`; the URDF itself declares `[-pi/2, pi/2]`.
Unlike v2, the last channel is a **prismatic joint**, so its target and converter
unit are metres rather than radians.

### What MATLAB sets on those joint blocks

For every name in `jointNames`, `configureContinuousInputs()` calls `set_param`
on the corresponding block in `dum_e_v3_live.slx`:

| Parameter | Value set by the runner |
|---|---|
| `MotionActuationMode` | `InputMotion` |
| `TorqueActuationMode` | `ComputedTorque` |
| `PositionTargetSpecify` | `on` |
| `PositionTargetPriority` | `High` |
| `PositionTargetValue` | the initial pose value from the JSON |
| `PositionTargetValueUnits` | `rad`, except `gripper_slide` which uses `m` |

`shoulder_pitch` additionally gets `LowerLimitBound = -pi` and
`UpperLimitBound = 0`. The clean generated source model leaves these joints in
`ComputedMotion` / `NoTorque` mode; only the live copy is actuated this way.

### The blocks that generate the signals

`configureContinuousInputs()` adds these blocks to the live model:

- `DumeLivePoseSource` — a Level-2 MATLAB S-function whose `FunctionName` is
  `follow_dume_v3`. Its parameters are
  `'<stateFile>', 0.02, <moveDuration>, -0.009, 0.01`: the JSON path, a 0.02 s
  (50 Hz) sample time, the transition duration, and the closed/open slider
  endpoints in metres. The S-function callback is `setupPoseSource()` in this
  same `.m` file.
- `DumeSplitPosition`, `DumeSplitVelocity`, `DumeSplitAcceleration` — three
  Demux blocks that split each 6-element vector into one scalar per joint.
- `DumeServoFilter1/2/3` — three continuous Transfer Fcn blocks used only for
  the slider channel. With `tau = max(0.01, moveDuration/10)` their denominator
  is `[tau^3 3*tau^2 3*tau 1]`, i.e. `(tau*s + 1)^3`, and their numerators are
  `[1]`, `[1 0]`, `[1 0 0]` for position, velocity and acceleration.
- `dumeUnits_<joint>` — six `nesl_utility/Simulink-PS Converter` blocks. Five
  use `Unit = 'rad'`; `dumeUnits_gripper_slide` uses `Unit = 'm'`. All use
  `FilteringAndDerivatives = 'provide'` and `UdotUserProvided = '2'`, so
  Simscape receives q, qdot and qddot explicitly.

Channels 1–5 are wired straight from the Demuxes to their converter inputs.
Channel 6 (the slider) is wired through the three filters first. That split is
the whole reason the arm can be driven directly while the closed four-loop
gripper stays numerically well behaved.

### What the Python script controls, in practice

Python controls the **value** delivered to an already-wired block, not the
choice of block. The field names, ranges and mapping live in `readDumePose()`
here in `follow_dume_v3.m`. If you rename a JSON field or want a different GUI
axis to drive a different CAD joint, you must change this MATLAB function (and
the corresponding `jointNames` entry), not the Python GUI alone.

The S-function polls the file every 20 ms. It waits at the last pose while the
file is unchanged; when a value changes it starts a quintic transition of
`moveDuration` seconds that ends with zero velocity and acceleration. During a
transition it keeps polling, so a second GUI change restarts the curve from the
current position, velocity and acceleration rather than jumping.

