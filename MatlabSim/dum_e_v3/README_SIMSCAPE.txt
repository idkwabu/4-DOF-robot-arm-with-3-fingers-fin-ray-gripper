DUM-E V3 MK19 SIMSCAPE PRELIMINARY EXPORT

The URDF is an open-loop tree containing 15 links, 13 revolute joints,
and the gripper slider. In MATLAB, make this folder current and run:
    import_dume_simscape
The helper restores four gripper closure joints from loop_constraints.json
and applies standard gravity in the Y-up frame, [0 -9.80665 0] m/s^2.
The base link is attached to the simulated world.

PRELIMINARY INERTIAS: CAD-assigned material/density is used, not independently verified.
Estimated total mass 164.431 kg; omitted 70 non-solid bodies and 9 invalid solids.
Overlapping solids may be double-counted; mesh tessellation is approximate.
See cad_inertia_estimates.json for per-link coverage and methods.
Replace mass, COM, and inertia with measured values before using dynamic results.

Effort/velocity values of 100 are required placeholders, not hardware ratings.
The slider limits come from Fusion and are converted from cm to m. Most gripper
hinge limits, actuator limits, and coupling behavior are unverified. Collision
meshes are unsimplified. Verify full travel and collisions before use.
