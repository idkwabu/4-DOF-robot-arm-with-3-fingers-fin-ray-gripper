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
