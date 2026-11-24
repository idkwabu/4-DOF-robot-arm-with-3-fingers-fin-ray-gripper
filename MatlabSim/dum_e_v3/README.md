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
