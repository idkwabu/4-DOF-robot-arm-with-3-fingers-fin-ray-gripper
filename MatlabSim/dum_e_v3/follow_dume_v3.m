function follow_dume_v3(startGui, stopTime)
%FOLLOW_DUME_V3 Simulate one snapshot of the current DUM-E v3 pose.
% Run once after a DUM-E command; rerun after the next command:
%   cd('D:\Dum-E\MatlabSim\dum_e_v3')
%   follow_dume_v3(false, 1)

if nargin < 1
    startGui = true;
end
if nargin < 2
    stopTime = 1;
end
validateattributes(startGui, {'logical','numeric'}, {'scalar'});
validateattributes(stopTime, {'numeric'}, {'scalar','real','positive','finite'});

gripperClosedMeters = -0.009;
gripperOpenMeters = 0.010;
jointNames = {
    'base_yaw'
    'shoulder_pitch'
    'elbow_pitch'
    'wrist_pitch'
    'wrist_roll'
    'gripper_slide'
};

modelFolder = fileparts(mfilename('fullpath'));
projectFolder = fileparts(fileparts(modelFolder));
stateFile = fullfile(projectFolder, 'robot_state.json');
guiFile = fullfile(projectFolder, 'Dum-E.py');
pythonExe = fullfile(projectFolder, '.venv', 'Scripts', 'python.exe');
oldFolder = cd(modelFolder);
folderCleanup = onCleanup(@() cd(oldFolder));

if startGui
    if ~isfile(pythonExe) || ~isfile(guiFile)
        error('Could not find Dum-E.py or the project virtual-environment Python interpreter.');
    end
    command = sprintf('start "" /D "%s" "%s" "%s"', ...
        projectFolder, pythonExe, guiFile);
    [status, output] = system(command);
    if status ~= 0
        error('Could not launch Dum-E.py: %s', output);
    end
end

waitForStateFile(stateFile, 30);
pose = read_dume_pose_v3( ...
    stateFile, gripperClosedMeters, gripperOpenMeters);
model = getDumeModel(modelFolder);
removeOldPoseInputs(model, jointNames);
set_param([model '/MechanismConfiguration'], 'GravityVector', '[0 0 0]');

for k = 1:numel(jointNames)
    joint = [model '/' jointNames{k}];
    set_param(joint, ...
        'MotionActuationMode', 'ComputedMotion', ...
        'TorqueActuationMode', 'NoTorque');
    if strcmp(jointNames{k}, 'shoulder_pitch')
        set_param(joint, ...
            'LowerLimitSpecify', 'on', ...
            'LowerLimitBound', sprintf('%.16g', -pi), ...
            'LowerLimitBoundUnits', 'rad', ...
            'UpperLimitSpecify', 'on', ...
            'UpperLimitBound', '0', ...
            'UpperLimitBoundUnits', 'rad');
    end
    units = 'rad';
    if strcmp(jointNames{k}, 'gripper_slide')
        units = 'm';
    end
    set_param(joint, ...
        'PositionTargetSpecify', 'on', ...
        'PositionTargetPriority', 'High', ...
        'PositionTargetValue', sprintf('%.16g', pose(k)), ...
        'PositionTargetValueUnits', units);
end

set_param(model, 'SimulationCommand', 'update');
set_param(model, ...
    'StopTime', sprintf('%.9g', stopTime), ...
    'EnablePacing', 'off', ...
    'MaxStep', '0.01');

fprintf(['Reading one pose snapshot from:\n%s\n' ...
    'Simulating this pose for %.3g seconds without real-time pacing.\n' ...
    'Run follow_dume_v3 again after the next DUM-E command.\n'], ...
    stateFile, stopTime);
sim(model);
fprintf('Snapshot simulation finished; the model remains open for inspection.\n');
end

function model = getDumeModel(modelFolder)
modelName = 'dum_e_simscape_generated';
if bdIsLoaded(modelName)
    if ~strcmp(get_param(modelName, 'SimulationStatus'), 'stopped')
        error('Stop the current %s simulation before running a pose snapshot.', modelName);
    end
    model = modelName;
elseif isfile(fullfile(modelFolder, [modelName '.slx']))
    load_system(fullfile(modelFolder, [modelName '.slx']));
    model = modelName;
else
    model = import_dume_simscape();
end
end

function waitForStateFile(stateFile, timeoutSeconds)
started = tic;
while ~isfile(stateFile)
    if toc(started) >= timeoutSeconds
        error('DUM-E did not create its pose file: %s', stateFile);
    end
    pause(0.1);
end
end

function removeOldPoseInputs(model, jointNames)
oldNames = {'DumeLivePoseSource','DumeLiveClock','DumeSplitPosition', ...
    'DumeSplitVelocity','DumeSplitAcceleration','DumeHoldPosition', ...
    'DumeHoldVelocity','DumeHoldAcceleration'};
for k = 1:numel(jointNames)
    oldNames(end+1:end+4) = { ...
        ['dumePosition_' jointNames{k}], ...
        ['dumeVelocity_' jointNames{k}], ...
        ['dumeAcceleration_' jointNames{k}], ...
        ['dumeUnits_' jointNames{k}]};
end

blocks = get_param(model, 'Blocks');
for k = 1:numel(oldNames)
    if any(strcmp(blocks, oldNames{k}))
        delete_block([model '/' oldNames{k}]);
    end
end
end
