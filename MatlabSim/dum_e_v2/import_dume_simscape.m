function model = import_dume_simscape()
% Import the exported DUM-E URDF and restore the three CAD gripper loops.
% Inertias, when supplied, are provisional and not a validated dynamic model.
folder = fileparts(mfilename('fullpath'));
if ~strcmpi(pwd, folder)
    error('Set the MATLAB current folder to the exported DUM-E folder before running this function.');
end
name = 'dum_e_simscape_generated';
if bdIsLoaded(name) || isfile(fullfile(folder, [name '.slx']))
    error('A model named %s is already open or saved in this folder. Rename or close it before importing.', name);
end
info = jsondecode(fileread(fullfile(folder, 'loop_constraints.json')));
model = smimport(fullfile(folder, 'dum_e_v2_mk16.urdf'), 'ModelName', name);
model = getfullname(model);
% The Fusion source is Y-up. Configure standard gravity in the -Y direction.
mc = [model '/MechanismConfiguration'];
set_param(mc, 'UniformGravity', 'Constant', ...
    'GravityVector', '[0 -9.80665 0]', 'GravityVectorUnits', 'm/s^2');
add_dume_v2_gripper_closures(model, info);
fprintf(['Imported %s and restored 3 gripper closures.\n' ...
    'IMPORTANT: Supply verified mass/inertia and actuator data before dynamic use.\n' ...
    'The source is Y-up. Gravity is set to [0 -9.80665 0] m/s^2.\n'], model);
end
