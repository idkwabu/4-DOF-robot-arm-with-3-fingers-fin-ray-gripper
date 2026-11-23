# DUM-E v3 MATLAB/Simscape

This folder contains the v3 Mk19 Simscape export and MATLAB tools that read
poses written by the repository's `Dum-E.py` GUI.

## Run one simulation per DUM-E command

In MATLAB, run:

```matlab
cd('D:\Dum-E\MatlabSim\dum_e_v3')
follow_dume_v3
```

By default, the script launches `Dum-E.py`, reads
`D:\Dum-E\robot_state.json` once, then runs a one-second simulation without
real-time pacing. The model stays open afterward for inspection. Run
`follow_dume_v3` again after each DUM-E command; pose changes made during a run
are read on the next run. If the GUI is already open, use
`follow_dume_v3(false, 1)`. The second argument sets simulated duration in
seconds. The one-shot runner disables gravity because the preliminary passive
gripper linkage may lose its loop closure under gravity; the importer itself
still sets Y-up gravity to `[0 -9.80665 0] m/s^2`.

The former continuous follower is retained as `follow_dume_v3_realtime`, but
is not the recommended mode while the gripper closure issue remains.

The runner imports the Simscape model and restores the four gripper loop
closures through `import_dume_simscape` if no saved
`dum_e_simscape_generated.slx` model is available.

## Pose mapping

| DUM-E control | v3 Simscape joint |
| --- | --- |
| Base angle | `base_yaw` = negative DUM-E base angle |
| Joint 1 | `shoulder_pitch` = DUM-E joint 1 minus 90 degrees |
| Joint 2 | `elbow_pitch`, unchanged |
| Joint 3 | `wrist_pitch`, unchanged |
| Gripper roll | `wrist_roll`, unchanged |
| Gripper opening (0-100%) | `gripper_slide`, linearly mapped from -9 mm (closed) to +10 mm (open) |

The base and shoulder directions preserve the corrections verified against the
v2 model. The shoulder limit is expanded to -180..0 degrees so the offset
represents DUM-E's full -90..90 degree range. The gripper endpoints follow the
URDF slider limits; their open/closed direction is an initial assumption
because the CAD export does not specify a verified actuator direction. If the
gripper moves oppositely in the simulation, swap the `gripperClosedMeters` and
`gripperOpenMeters` values in `follow_dume_v3.m`.

The v3 CAD gripper is a two-jaw linkage, whereas the DUM-E GUI depicts a
three-finger gripper. The shared opening percentage therefore drives v3 slider
travel, not an identical finger motion.

## Files

- `import_dume_simscape.m` imports the v3 URDF, restores CAD loop closures,
  and sets gravity for the Y-up model.
- `follow_dume_v3.m` reads one pose snapshot and runs a finite simulation.
- `follow_dume_v3_realtime.m` retains the continuous follower.
- `DumePoseSourceV3.m` polls the pose file and generates trajectories for the
  continuous follower.
- `read_dume_pose_v3.m` validates the JSON pose and applies the mapping.
- `dum_e_v3_mk19.urdf`, `meshes/`, and `loop_constraints.json` are the model
  source assets. `dum_e_v3_generated.slx` is preserved and is not overwritten
  by the runner.
- `README_SIMSCAPE.txt` and `cad_inertia_estimates.json` document the export's
  existing physical-model caveats.

## Model limitations

The CAD inertias and estimated 164.4 kg total mass are preliminary, use
unverified steel density, and may double-count overlapping solids. Placeholder
effort/velocity limits are not hardware specifications. The slider range is
from Fusion, but gripper hinge limits and coupling behavior are unverified.
Treat this as a kinematic/visualization integration until the physical model
and gripper direction are checked.
