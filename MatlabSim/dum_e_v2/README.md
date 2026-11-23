# DUM-E v2 MATLAB / Simscape

This folder contains the v2 robot-arm model and MATLAB tools for importing it,
setting a pose, or following commands from the DUM-E Python GUI. The imported
model uses the Fusion coordinate convention: **Y is up**, with gravity set to
`[0 -9.80665 0] m/s^2`.

## Live control from DUM-E

From MATLAB:

```matlab
cd('D:\Dum-E\MatlabSim\dum_e_v2')
follow_dume_v2
```

By default, this launches `Dum-E.py` using the repository `.venv`, then runs
one continuous, real-time-paced Simscape simulation. DUM-E writes its current
pose to `D:\Dum-E\robot_state.json`; MATLAB reads that file while the
simulation is running. New poses are interpolated over 0.8 seconds and do not
restart the simulation. The live follower disables gravity because the
preliminary passive gripper linkage can lose its loop closure under gravity.
The importer itself still sets Y-up gravity to `[0 -9.80665 0] m/s^2`.
Press **Ctrl+C** in MATLAB to stop.

If DUM-E is already open, use `follow_dume_v2(false)`. A finite run can be
requested with `follow_dume_v2(false, 10)`, where the second argument is the
simulation stop time in seconds.

### Pose mapping

`read_dume_pose.m` validates the JSON pose and converts it to Simscape joint
positions in radians:

| DUM-E value | Simscape joint mapping |
|---|---|
| `base` | `-base` |
| `joint1` | `joint1 - 90 deg` |
| `joint2` | unchanged |
| `joint3` | unchanged |
| `gripper_roll` | unchanged |
| `gripper` (0-100%) | Linear mapping from -120 deg (closed) to +65 deg (open) |

The shoulder joint limits are adjusted to -180..0 deg to preserve DUM-E's full
joint1 range after the offset. `DumePoseSource.m` polls the state every 0.02 s
(50 Hz) and generates smooth position, velocity, and acceleration commands.

## Other MATLAB entry points

### Import the Simscape model

```matlab
cd('D:\Dum-E\MatlabSim\dum_e_v2')
model = import_dume_simscape();
```

The importer reads `dum_e_v2_mk16.urdf`, restores the three CAD gripper closure
joints described in `loop_constraints.json`, sets gravity, updates the model,
and leaves it open. It requires the MATLAB current folder to be this folder.
It refuses to import if `dum_e_simscape_generated` is already open or saved.

### Set a static pose

```matlab
run_dume_v2_simulation([45 30 -20 -30 0 -30])
```

The six values are Simscape joint target angles in degrees, in this order:
base yaw, shoulder, elbow, wrist pitch, wrist roll, and gripper servo. This
imports the model and sets its joint position targets; it does not run a
time-based motion or read DUM-E's state file.

## Files

| File | Purpose |
|---|---|
| `dum_e_v2_mk16.urdf` | Robot links, joints, visual/collision meshes, and preliminary inertias |
| `meshes/` | STL meshes referenced by the URDF |
| `loop_constraints.json` | CAD frames used to restore three gripper linkage closures |
| `cad_inertia_estimates.json` | Per-link details and caveats for estimated inertias |
| `import_dume_simscape.m` | Imports the URDF and adds the gripper closure joints |
| `follow_dume_v2.m` | Configures and runs the continuous DUM-E-controlled simulation |
| `DumePoseSource.m` | Polls the pose file and generates smooth joint trajectories |
| `read_dume_pose.m` | Validates and maps DUM-E pose values to Simscape radians |
| `run_dume_v2_simulation.m` | Sets a one-time static joint target pose |
| `README_SIMSCAPE.txt` | Original export notes and model caveats |
| `dum_e_v2_generated.slx` | Simulink/Simscape model generated with the export |
| `dum_e_v2_generated.avi` | Previously generated model animation |

MATLAB may also create `dum_e_simscape_generated.slx` and a `slprj` cache when
the importer or live follower runs. The live follower loads the generated
model if it exists and rebuilds its DUM-E input blocks.

## Model limitations

The URDF's CAD inertias are **preliminary estimates**, not measured physical
properties. The export assigns Steel density and estimates a total robot mass
of about 164.469 kg; excluded or overlapping CAD bodies may make this
inaccurate. Joint effort and velocity values of 100 are schema placeholders,
not actuator ratings. Verify masses, limits, gripper closure, collisions, and
motion against the physical arm before trusting dynamic results. This folder
controls only the MATLAB simulation; it does not send commands to robot
hardware. The live follower is intended for visual pose tracking, not
gravity-loaded dynamics.
