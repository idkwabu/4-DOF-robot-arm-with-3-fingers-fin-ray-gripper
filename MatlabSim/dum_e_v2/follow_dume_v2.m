function follow_dume_v2(startGui, stopTime)
%FOLLOW_DUME_V2 Simulate one snapshot of the current DUM-E v2 pose.
% Run once after a DUM-E command; rerun after the next command:
%   cd('D:\Dum-E\MatlabSim\dum_e_v2')
%   follow_dume_v2(false, 1)

if nargin < 1
    startGui = true;
end
if nargin < 2
    stopTime = 1;
end
validateattributes(startGui, {'logical','numeric'}, {'scalar'});
validateattributes(stopTime, {'numeric'}, {'scalar','real','positive','finite'});

gripperClosedDeg = -120;
gripperOpenDeg = 65;
jointNames = {
    'base_yaw_joint'
    'shoulder_pitch_joint'
    'elbow_pitch_joint'
    'wrist_pitch_joint'
    'wrist_roll_joint'
    'gripper_servo_pivot_joint'
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
pose = read_dume_pose(stateFile, gripperClosedDeg, gripperOpenDeg);
model = getDumeModel(modelFolder);
removeOldPoseInputs(model, jointNames);
set_param([model '/MechanismConfiguration'], 'GravityVector', '[0 0 0]');

configureSnapshotInputs(model, jointNames, pose);

set_param(model, 'SimulationCommand', 'update');
set_param(model, ...
    'StopTime', sprintf('%.9g', stopTime), ...
    'EnablePacing', 'off', ...
    'MaxStep', '0.01');

fprintf(['Reading one pose snapshot from:\n%s\n' ...
    'Holding that pose for %.3g simulated seconds without real-time pacing.\n' ...
    'Run follow_dume_v2 again after the next DUM-E command.\n'], ...
    stateFile, stopTime);
sim(model);
fprintf('Snapshot simulation finished; the model remains open for inspection.\n');
end

function configureSnapshotInputs(model, jointNames, pose)
for k = 1:numel(jointNames)
    joint = [model '/' jointNames{k}];
    units = 'rad';
    set_param(joint, ...
        'MotionActuationMode', 'InputMotion', ...
        'TorqueActuationMode', 'ComputedTorque', ...
        'PositionTargetSpecify', 'on', ...
        'PositionTargetPriority', 'High', ...
        'PositionTargetValue', sprintf('%.16g', pose(k)), ...
        'PositionTargetValueUnits', units);
    if strcmp(jointNames{k}, 'shoulder_pitch_joint')
        set_param(joint, ...
            'LowerLimitSpecify', 'on', ...
            'LowerLimitBound', sprintf('%.16g', -pi), ...
            'LowerLimitBoundUnits', 'rad', ...
            'UpperLimitSpecify', 'on', ...
            'UpperLimitBound', '0', ...
            'UpperLimitBoundUnits', 'rad');
    end

    constantName = ['DumeSnapshot_' jointNames{k}];
    converterName = ['dumeSnapshotPS_' jointNames{k}];
    constant = [model '/' constantName];
    converter = [model '/' converterName];
    blocks = get_param(model, 'Blocks');
    hasConstant = any(strcmp(blocks, constantName));
    hasConverter = any(strcmp(blocks, converterName));

    if hasConstant && hasConverter
        set_param(constant, 'Value', sprintf('%.16g', pose(k)));
        set_param(converter, 'Unit', units);
        continue;
    end
    if hasConstant
        delete_block(constant);
    end
    if hasConverter
        delete_block(converter);
    end
    add_block('simulink/Sources/Constant', constant, ...
        'Value', sprintf('%.16g', pose(k)), ...
        'Position', [700 80+100*(k-1) 760 110+100*(k-1)]);
    add_block('nesl_utility/Simulink-PS Converter', converter, ...
        'Unit', units, ...
        'Position', [820 80+100*(k-1) 900 110+100*(k-1)]);

    constantPorts = get_param(constant, 'PortHandles');
    converterPorts = get_param(converter, 'PortHandles');
    jointPorts = get_param(joint, 'PortHandles');
    add_line(model, constantPorts.Outport(1), converterPorts.Inport(1), ...
        'autorouting', 'on');
    add_line(model, converterPorts.RConn(1), jointPorts.LConn(2), ...
        'autorouting', 'on');
end
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
        ['dumeRadians_' jointNames{k}]};
end

blocks = get_param(model, 'Blocks');
for k = 1:numel(oldNames)
    if any(strcmp(blocks, oldNames{k}))
        delete_block([model '/' oldNames{k}]);
    end
end
end
