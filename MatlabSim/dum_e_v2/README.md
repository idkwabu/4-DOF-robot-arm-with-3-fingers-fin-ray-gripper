# DUM-E v2 MATLAB/Simscape

The only MATLAB entry point is `follow_dume_v2.m`. It contains the live
follower, state-file validation and pose mapping, Simulink pose source, model
import, and gripper closure helpers.

Run the simulation from MATLAB:

```matlab
cd('D:\Dum-E\MatlabSim\dum_e_v2')
follow_dume_v2
```

By default this launches `Dum-E.py`, opens the Simulink model and Simscape
Mechanics Explorer, then follows `D:\Dum-E\robot_state.json`. It waits at the
current pose while the file is unchanged and animates each new pose over 0.5
simulated seconds. Press **Ctrl+C** in MATLAB to stop.

If the Python GUI is already open:

```matlab
follow_dume_v2(false)
```

Optional arguments are `startGui`, `stopTime`, `moveDuration`, and `stateFile`;
for example:

```matlab
follow_dume_v2(false, 120, 0.5, 'D:\Dum-E\test_pose.json')
```

Each run saves the configured live model as `dum_e_simscape_live.slx` in this
folder, overwriting the previous copy, and leaves it open in Mechanics
Explorer. The clean source model `dum_e_simscape_generated.slx` is rebuilt from
the URDF automatically if it is missing or stale; you can force a rebuild with:

```matlab
follow_dume_v2('rebuildSource')
```

Live gripper animation preserves the original CAD linkage: the three
rod-to-finger closure joints and their frame helpers are kept, and only the
gripper servo is driven. The closures are **spherical joints** (ball joints)
rather than revolute joints — the rod ends stay attached to the finger
frames, but the redundant axis-alignment constraint is removed, which made
the three-loop mechanism singular and stopped the simulation whenever the
gripper moved. Live mode disables gravity because the preliminary passive
linkage can lose closure under gravity; the clean source model retains Y-up
gravity. The sampled pose source is smoothed for the gripper servo by three
continuous transfer-function filters (position, velocity, acceleration); a
sampled-and-held servo command loses the rod/finger constraints. The five arm
joints keep their original direct command path.

The GUI's gripper opening (`0..1`, written as `gripper: 0..100`) is mapped to
the servo range `0..59.3 deg`. That range was calibrated against the CAD
mechanism: servo `0 deg` holds the fingers at `0 deg` (closed, matching the
Python model), and servo `59.3 deg` gives about `30 deg` of finger swing,
matching `FINGER_OPENING_DEG` in `movements/robot_arm.py`. Do not extend the
servo below `0 deg` — the linkage reaches a singularity near `-60 deg` where
one finger flips and the solver can fail and stop the simulation.

The source model is Y-up and uses `[0 -9.80665 0] m/s^2` gravity when imported.
Its inertias are preliminary estimates, not measured physical properties.
This simulation does not send commands to robot hardware.

---

## How the Python pose reaches a Simulink joint

The Python GUI does **not** talk to Simulink directly. It never names a block
and it never sets a block parameter. Its whole contribution is to write six
validated numbers to `D:\Dum-E\robot_state.json`. The MATLAB runner owns the
wiring: `follow_dume_v2.m` reads that file, converts the numbers to CAD units,
and `configureContinuousInputs()` decides which Simulink joint block receives
each value.

```text
Dum-E.py
   | writes base, joint1, joint2, joint3, gripper_roll, gripper
   v
robot_state.json
   | read by the DumeLivePoseSource S-function inside the .slx
   v
readDumePose()  -> radians + joint mapping
   v
DumeLivePoseSource  -> q, qdot, qddot (6-element vectors)
   v
DumeSplitPosition / DumeSplitVelocity / DumeSplitAcceleration
   v
dumeRadians_<joint> Simulink-PS Converters
   v
revolute / servo joint blocks
```

### Which JSON field drives which block

`readDumePose()` validates the six fields against
`[-180 180; -90 90; -150 150; -120 120; -180 180; 0 100]` and then applies:

```matlab
armPoseDeg = values(1:5) .* [-1 1 1 1 1] + [0 -90 0 0 0];
pose = deg2rad([armPoseDeg gripperServoDeg]).';
```

| `robot_state.json` field | Conversion | Joint block in the live model | Signal path |
|---|---|---|---|
| `base` | negated, then deg → rad | `base_yaw_joint` | direct q/qd/qdd |
| `joint1` | `-90 deg`, then deg → rad | `shoulder_pitch_joint` | direct q/qd/qdd |
| `joint2` | deg → rad | `elbow_pitch_joint` | direct q/qd/qdd |
| `joint3` | deg → rad | `wrist_pitch_joint` | direct q/qd/qdd |
| `gripper_roll` | deg → rad | `wrist_roll_joint` | direct q/qd/qdd |
| `gripper` | `0..100` → `0..59.3 deg` → rad | `gripper_servo_pivot_joint` | **filtered** q/qd/qdd |

The `-90 deg` shift on `joint1` is why the runner overrides that joint's limits
in the live model to `[-pi, 0] rad`; the URDF itself declares `[-pi/2, pi/2]`.

### What MATLAB sets on those joint blocks

For every name in `jointNames`, `configureContinuousInputs()` calls `set_param`
on the corresponding block in `dum_e_simscape_live.slx`:

| Parameter | Value set by the runner |
|---|---|
| `MotionActuationMode` | `InputMotion` |
| `TorqueActuationMode` | `ComputedTorque` |
| `PositionTargetSpecify` | `on` |
| `PositionTargetPriority` | `High` |
| `PositionTargetValue` | the initial pose value from the JSON (radians) |
| `PositionTargetValueUnits` | `rad` |

`shoulder_pitch_joint` additionally gets `LowerLimitBound = -pi` and
`UpperLimitBound = 0`. The clean generated source model leaves these joints in
`ComputedMotion` / `NoTorque` mode; only the live copy is actuated this way.

### The blocks that generate the signals

`configureContinuousInputs()` adds these blocks to the live model:

- `DumeLivePoseSource` — a Level-2 MATLAB S-function whose `FunctionName` is
  `follow_dume_v2`. Its parameters are
  `'<stateFile>', 0.02, <moveDuration>, 0, 59.3`: the JSON path, a 0.02 s
  (50 Hz) sample time, the transition duration, and the closed/open servo
  endpoints in degrees. The S-function callback is `setupPoseSource()` in this
  same `.m` file.
- `DumeSplitPosition`, `DumeSplitVelocity`, `DumeSplitAcceleration` — three
  Demux blocks that split each 6-element vector into one scalar per joint.
- `DumeServoFilter1/2/3` — three continuous Transfer Fcn blocks used only for
  the gripper servo channel. With `tau = max(0.01, moveDuration/10)` their
  denominator is `[tau^3 3*tau^2 3*tau 1]`, i.e. `(tau*s + 1)^3`, and their
  numerators are `[1]`, `[1 0]`, `[1 0 0]` for position, velocity and
  acceleration.
- `dumeRadians_<joint>` — six `nesl_utility/Simulink-PS Converter` blocks. Each
  is configured with `Unit = 'rad'`, `FilteringAndDerivatives = 'provide'`, and
  `UdotUserProvided = '2'`, so Simscape receives q, qdot and qddot explicitly.

Channels 1–5 are wired straight from the Demuxes to their converter inputs.
Channel 6 (the servo) is wired through the three filters first. That split is
the whole reason the arm can be driven directly while the closed three-loop
gripper stays numerically well behaved.

### What the Python script controls, in practice

Python controls the **value** delivered to an already-wired block, not the
choice of block. The field names, ranges and mapping live in `readDumePose()`
here in `follow_dume_v2.m`. If you rename a JSON field or want a different GUI
axis to drive a different CAD joint, you must change this MATLAB function (and
the corresponding `jointNames` entry), not the Python GUI alone.

The S-function polls the file every 20 ms. It waits at the last pose while the
file is unchanged; when a value changes it starts a quintic transition of
`moveDuration` seconds that ends with zero velocity and acceleration. During a
transition it keeps polling, so a second GUI change restarts the curve from the
current position, velocity and acceleration rather than jumping.

