function run_dume_v2_simulation(poseDeg)
%RUN_DUME_V2_SIMULATION Import and configure one stable DUM-E v2 pose.
%
% poseDeg columns are: base, shoulder, elbow, wrist pitch, wrist roll,
% gripper servo. All values are in degrees.

if nargin < 1
    poseDeg = [45 30 -20 -30 0 -30];
end

if numel(poseDeg) ~= 6
    error('poseDeg must contain exactly six joint angles.');
end

poseDeg = poseDeg(:).';

folder = fileparts(mfilename('fullpath'));
oldFolder = cd(folder);
cleanup = onCleanup(@() cd(oldFolder));

model = import_dume_simscape();

jointNames = {
    'base_yaw_joint'
    'shoulder_pitch_joint'
    'elbow_pitch_joint'
    'wrist_pitch_joint'
    'wrist_roll_joint'
    'gripper_servo_pivot_joint'
};

for k = 1:numel(jointNames)
    block = [model '/' jointNames{k}];
    set_param(block, ...
        'PositionTargetSpecify', 'on', ...
        'PositionTargetPriority', 'High', ...
        'PositionTargetValue', sprintf('%.9g', deg2rad(poseDeg(k))), ...
        'PositionTargetValueUnits', 'rad');
end

set_param(model, 'SimulationCommand', 'update');
fprintf('DUM-E v2 model updated for pose [');
fprintf(' %.1f', poseDeg);
fprintf(' ] degrees.\n');
end
