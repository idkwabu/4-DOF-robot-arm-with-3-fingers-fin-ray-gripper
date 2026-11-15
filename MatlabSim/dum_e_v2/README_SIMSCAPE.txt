DUM-E SIMSCAPE PRELIMINARY EXPORT

The URDF itself is an open-loop tree. In MATLAB, set the current folder to
this export folder and run:
    import_dume_simscape
The helper restores the three gripper closure joints using the joint frames
in loop_constraints.json. The Fusion design is not modified. Eleven legacy
joints with missing endpoints are not included.

PRELIMINARY INERTIAS: The source assigns Steel at 7850 kg/m^3 to every body.
The resulting estimated robot mass is 164.469 kg. These are not verified physical values.
Seventy non-solid bodies and six solids with invalid mesh inertias were omitted.
Overlapping solid bodies may be counted more than once. STL tessellation is approximate.
See cad_inertia_estimates.json for per-link coverage and methods.
Replace masses, centers of mass, and inertia tensors with real data before using dynamic results.

The joint effort and velocity values of 100 are schema placeholders, not
verified actuator ratings. Several gripper limits are unknown. The source is
Y-up and STL meshes are millimeters. The MATLAB helper sets gravity along -Y;
if the base is rotated into a Z-up world, update gravity. Collision meshes
are unsimplified. Verify full
travel, closures and collisions before use.
