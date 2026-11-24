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

This launches `Dum-E.py` using the repository `.venv`, opens the Simulink model
and Simscape Mechanics Explorer, then follows
`D:\Dum-E\robot_state.json` in one continuous simulation. It waits at the
current pose while the file is unchanged, then paces each new move for visible
animation over 0.5 simulated seconds without restarting the simulation. State
changes during a move retarget the trajectory smoothly. To avoid the CAD
gripper loops failing during motion, live mode temporarily removes the three
rod-to-finger closure joints and drives the three finger pivots independently
through 0-40 degrees. The rods therefore are not mechanically linked to the
fingers during live animation. Live mode runs in a temporary copy of the
generated model, so the imported model retains its full CAD linkage. Live
mode also disables gravity because the preliminary passive gripper linkage
can lose its loop closure under gravity; the importer itself still sets Y-up
gravity to `[0 -9.80665 0] m/s^2`.
Press **Ctrl+C** in MATLAB to stop.

If DUM-E is already open, use `follow_dume_v2(false)`. The optional second
argument sets simulation duration, and the third adjusts animation duration,
e.g. `follow_dume_v2(false, 120, 0.5)`. The default run waits for state
changes until stopped. An optional fourth argument can select a different
pose JSON file, e.g.
`follow_dume_v2(false, 120, 0.5, 'D:\Dum-E\test_pose.json')`.

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
joint1 range after the offset.

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

### Run a separate pose snapshot

Use `follow_dume_v2_snapshot(false, 1)` to read the current state once and run
a finite, unpaced simulation. It will not follow state changes during that
run.

## Files

| File | Purpose |
|---|---|
| `dum_e_v2_mk16.urdf` | Robot links, joints, visual/collision meshes, and preliminary inertias |
| `meshes/` | STL meshes referenced by the URDF |
| `loop_constraints.json` | CAD frames used to restore three gripper linkage closures |
| `cad_inertia_estimates.json` | Per-link details and caveats for estimated inertias |
| `import_dume_simscape.m` | Imports the URDF and adds the gripper closure joints |
| `follow_dume_v2.m` | Opens the GUI and continuously follows DUM-E |
| `follow_dume_v2_snapshot.m` | Reads one pose and runs a finite simulation |
| `add_dume_v2_gripper_closures.m` | Restores the CAD gripper loop joints |
| `DumePoseSource.m` | Polls the pose file and generates smooth joint trajectories |
| `read_dume_pose.m` | Validates and maps DUM-E pose values to Simscape radians |
| `run_dume_v2_simulation.m` | Sets a one-time static joint target pose |
| `README_SIMSCAPE.txt` | Original export notes and model caveats |
| `dum_e_v2_generated.slx` | Simulink/Simscape model generated with the export |
| `dum_e_v2_generated.avi` | Previously generated model animation |

MATLAB may also create `dum_e_simscape_generated.slx` and a `slprj` cache when
the importer or a runner executes.

## Model limitations

The URDF's CAD inertias are **preliminary estimates**, not measured physical
properties. The export assigns Steel density and estimates a total robot mass
of about 164.469 kg; excluded or overlapping CAD bodies may make this
inaccurate. Joint effort and velocity values of 100 are schema placeholders,
not actuator ratings. Verify masses, limits, gripper closure, collisions, and
motion against the physical arm before trusting dynamic results. This folder
controls only the MATLAB simulation; it does not send commands to robot
hardware. The one-shot runner is intended for pose visualization, not
gravity-loaded dynamics.
