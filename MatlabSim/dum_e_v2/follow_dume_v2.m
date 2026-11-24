function follow_dume_v2(startGui, stopTime, moveDuration, stateFile)
%FOLLOW_DUME_V2 Continuously animate DUM-E poses in Simscape.
% Run in MATLAB with:
%   cd('D:\Dum-E\MatlabSim\dum_e_v2')
%   follow_dume_v2
% Optional: follow_dume_v2(true, 120, 0.5) sets stop time and move duration.
% Simulation waits at each pose until the state file changes.

if nargin < 1
    startGui = true;
end
if nargin < 2
    stopTime = 1e6;
end
if nargin < 3
    moveDuration = 0.5;
end
if nargin < 4
    stateFile = '';
end
validateattributes(startGui, {'logical','numeric'}, {'scalar'});
validateattributes(stopTime, {'numeric'}, {'scalar','real','positive','finite'});
validateattributes(moveDuration, {'numeric'}, {'scalar','real','positive','finite'});

gripperClosedDeg = -120;
gripperOpenDeg = 65;
gripperFingerOpenDeg = 40;
jointNames = {
    'base_yaw_joint'
    'shoulder_pitch_joint'
    'elbow_pitch_joint'
    'wrist_pitch_joint'
    'wrist_roll_joint'
    'gripper_servo_pivot_joint'
    'gripper_finger_1_pivot_joint'
    'gripper_finger_2_pivot_joint'
    'gripper_finger_3_pivot_joint'
};

modelFolder = fileparts(mfilename('fullpath'));
projectFolder = fileparts(fileparts(modelFolder));
if isempty(stateFile)
    stateFile = fullfile(projectFolder, 'robot_state.json');
elseif isstring(stateFile) && isscalar(stateFile)
    stateFile = char(stateFile);
elseif ~ischar(stateFile) || ~isrow(stateFile)
    error('The optional state-file argument must be a character vector or scalar string.');
end
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
[initialPose, initialState] = read_dume_pose( ...
    stateFile, gripperClosedDeg, gripperOpenDeg);
initialFingerAngle = deg2rad( ...
    gripperFingerOpenDeg*(initialState.gripper/100));
initialPose = [initialPose; repmat(initialFingerAngle, 3, 1)];

sourceModelName = 'dum_e_simscape_generated';
sourceModelFile = fullfile(modelFolder, [sourceModelName '.slx']);
if bdIsLoaded(sourceModelName) && ...
        ~strcmp(get_param(sourceModelName, 'SimulationStatus'), 'stopped')
    error('Stop the existing %s simulation before starting the DUM-E follower.', ...
        sourceModelName);
end
if ~isfile(sourceModelFile)
    if ~bdIsLoaded(sourceModelName)
        import_dume_simscape();
    end
    if ~isfile(sourceModelFile)
        save_system(sourceModelName, sourceModelFile);
    end
end

[~, liveToken] = fileparts(tempname);
liveModelName = ['dum_e_simscape_live_' ...
    regexprep(liveToken, '[^A-Za-z0-9_]', '_')];
liveModelFile = fullfile(tempdir, [liveModelName '.slx']);
copyfile(sourceModelFile, liveModelFile);
liveModelCleanup = onCleanup(@() ...
    closeAndDeleteLiveModel(liveModelName, liveModelFile));
load_system(liveModelFile);
model = liveModelName;

removeGripperLoopConstraints(model);
set_param([model '/MechanismConfiguration'], 'GravityVector', '[0 0 0]');
configureContinuousInputs(model, jointNames, initialPose, stateFile, ...
    gripperClosedDeg, gripperOpenDeg, gripperFingerOpenDeg, moveDuration);
set_param(model, ...
    'StopTime', sprintf('%.9g', stopTime), ...
    'EnablePacing', 'on', ...
    'PacingRate', '1', ...
    'MaxStep', '0.01');

open_system(model);
fprintf(['Starting one continuous Simscape run; DUM-E poses are sampled from:\n%s\n' ...
    'Gripper 0-100%% maps to the v2 servo range %.0f to %.0f degrees.\n' ...
    'Live finger pivots move independently through %.0f degrees to avoid loop-closure failures.\n' ...
    'Simulation waits between poses and paces each move for visible animation over %.2f simulated seconds.\n' ...
    'Gravity is disabled for stable live pose tracking.\n' ...
    'Press Ctrl+C in MATLAB to stop.\n'], ...
    stateFile, gripperClosedDeg, gripperOpenDeg, gripperFingerOpenDeg, moveDuration);
sim(model);
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

function configureContinuousInputs( ...
        model, jointNames, initialPose, stateFile, ...
        gripperClosedDeg, gripperOpenDeg, gripperFingerOpenDeg, moveDuration)
removeOldPoseInputs(model, jointNames);

for k = 1:numel(jointNames)
    joint = [model '/' jointNames{k}];
    set_param(joint, ...
        'MotionActuationMode', 'ComputedMotion', ...
        'TorqueActuationMode', 'NoTorque');
    if strcmp(jointNames{k}, 'shoulder_pitch_joint')
        set_param(joint, ...
            'LowerLimitSpecify', 'on', ...
            'LowerLimitBound', sprintf('%.16g', -pi), ...
            'LowerLimitBoundUnits', 'rad', ...
            'UpperLimitSpecify', 'on', ...
            'UpperLimitBound', '0', ...
            'UpperLimitBoundUnits', 'rad');
    end
end
set_param(model, 'SimulationCommand', 'update');

for k = 1:numel(jointNames)
    joint = [model '/' jointNames{k}];
    set_param(joint, ...
        'MotionActuationMode', 'InputMotion', ...
        'TorqueActuationMode', 'ComputedTorque', ...
        'PositionTargetSpecify', 'on', ...
        'PositionTargetPriority', 'High', ...
        'PositionTargetValue', sprintf('%.16g', initialPose(k)), ...
        'PositionTargetValueUnits', 'rad');
end
set_param(model, 'SimulationCommand', 'update');

add_block('simulink/User-Defined Functions/MATLAB System', ...
    [model '/DumeLivePoseSource'], ...
    'Position', [700 80 850 150]);
set_param([model '/DumeLivePoseSource'], ...
    'System', 'DumePoseSource', ...
    'SimulateUsing', 'Interpreted execution', ...
    'StateFile', stateFile, ...
    'SampleTime', '0.02', ...
    'MoveDuration', sprintf('%.16g', moveDuration), ...
    'GripperClosedDeg', sprintf('%.16g', gripperClosedDeg), ...
    'GripperOpenDeg', sprintf('%.16g', gripperOpenDeg), ...
    'GripperFingerOpenDeg', sprintf('%.16g', gripperFingerOpenDeg));

add_block('simulink/Sources/Clock', [model '/DumeLiveClock'], ...
    'Position', [570 90 600 120]);
add_block('simulink/Signal Routing/Demux', [model '/DumeSplitPosition'], ...
    'Outputs', num2str(numel(jointNames)), 'Position', [890 75 895 190]);
add_block('simulink/Signal Routing/Demux', [model '/DumeSplitVelocity'], ...
    'Outputs', num2str(numel(jointNames)), 'Position', [890 215 895 330]);
add_block('simulink/Signal Routing/Demux', [model '/DumeSplitAcceleration'], ...
    'Outputs', num2str(numel(jointNames)), 'Position', [890 355 895 470]);
add_block('simulink/Discrete/Unit Delay', [model '/DumeHoldPosition'], ...
    'Position', [925 90 965 120]);
add_block('simulink/Discrete/Unit Delay', [model '/DumeHoldVelocity'], ...
    'Position', [925 230 965 260]);
add_block('simulink/Discrete/Unit Delay', [model '/DumeHoldAcceleration'], ...
    'Position', [925 370 965 400]);
set_param([model '/DumeHoldPosition'], ...
    'InitialCondition', mat2str(initialPose, 16));
set_param([model '/DumeHoldVelocity'], 'InitialCondition', 'zeros(9,1)');
set_param([model '/DumeHoldAcceleration'], 'InitialCondition', 'zeros(9,1)');

clockPorts = get_param([model '/DumeLiveClock'], 'PortHandles');
sourcePorts = get_param([model '/DumeLivePoseSource'], 'PortHandles');
add_line(model, clockPorts.Outport(1), sourcePorts.Inport(1), 'autorouting', 'on');

holdNames = {'DumeHoldPosition','DumeHoldVelocity','DumeHoldAcceleration'};
splitNames = {'DumeSplitPosition','DumeSplitVelocity','DumeSplitAcceleration'};
for k = 1:3
    holdPath = [model '/' holdNames{k}];
    splitPath = [model '/' splitNames{k}];
    holdPorts = get_param(holdPath, 'PortHandles');
    splitPorts = get_param(splitPath, 'PortHandles');
    add_line(model, sourcePorts.Outport(k), holdPorts.Inport(1), 'autorouting', 'on');
    add_line(model, holdPorts.Outport(1), splitPorts.Inport(1), 'autorouting', 'on');
end

q = get_param([model '/DumeSplitPosition'], 'PortHandles');
qd = get_param([model '/DumeSplitVelocity'], 'PortHandles');
qdd = get_param([model '/DumeSplitAcceleration'], 'PortHandles');
for k = 1:numel(jointNames)
    jointName = jointNames{k};
    y = 90 + 120*(k-1);
    converter = [model '/dumeRadians_' jointName];
    add_block('nesl_utility/Simulink-PS Converter', converter, ...
        'Unit', 'rad', ...
        'FilteringAndDerivatives', 'provide', ...
        'UdotUserProvided', '2', ...
        'Position', [1010 y 1085 y+55]);

    converterPorts = get_param(converter, 'PortHandles');
    jointPorts = get_param([model '/' jointName], 'PortHandles');
    add_line(model, q.Outport(k), converterPorts.Inport(1), 'autorouting', 'on');
    add_line(model, qd.Outport(k), converterPorts.Inport(2), 'autorouting', 'on');
    add_line(model, qdd.Outport(k), converterPorts.Inport(3), 'autorouting', 'on');
    add_line(model, converterPorts.RConn(1), jointPorts.LConn(2), 'autorouting', 'on');
end
set_param(model, 'SimulationCommand', 'update');
end

function removeOldPoseInputs(model, jointNames)
oldNames = {'DumeLivePoseSource','DumeLiveClock','DumeSplitPosition', ...
    'DumeSplitVelocity','DumeSplitAcceleration','DumeHoldPosition', ...
    'DumeHoldVelocity','DumeHoldAcceleration'};
oldNames(end+1:end+4*numel(jointNames)) = cell(1,4*numel(jointNames));
for k = 1:numel(jointNames)
    first = 8 + 4*(k-1) + 1;
    oldNames(first:first+3) = { ...
        ['dumePosition_' jointNames{k}], ...
        ['dumeVelocity_' jointNames{k}], ...
        ['dumeAcceleration_' jointNames{k}], ...
        ['dumeRadians_' jointNames{k}]};
    oldNames(end+1:end+2) = { ...
        ['DumeSnapshot_' jointNames{k}], ...
        ['dumeSnapshotPS_' jointNames{k}]};
end

blocks = get_param(model, 'Blocks');
for k = 1:numel(oldNames)
    if any(strcmp(blocks, oldNames{k}))
        delete_block([model '/' oldNames{k}]);
    end
end
end

function removeGripperLoopConstraints(model)
closureNames = {
    'cad_closure_rod_1_finger_1_joint'
    'cad_closure_rod_2_finger_3_joint'
    'cad_closure_rod_3_finger_2_joint'
};
blocks = get_param(model, 'Blocks');
for k = 1:numel(closureNames)
    if any(strcmp(blocks, closureNames{k}))
        delete_block([model '/' closureNames{k}]);
    end
end

closureLinks = {
    'gripper_connecting_rod_1_1'
    'gripper_finger_1_1'
    'gripper_connecting_rod_2_1'
    'gripper_finger_3_1'
    'gripper_connecting_rod_3_1'
    'gripper_finger_2_1'
};
for k = 1:numel(closureLinks)
    for blockName = {'ClosurePort','ClosureTransform'}
        block = [model '/' closureLinks{k} '/' blockName{1}];
        if getSimulinkBlockHandle(block) ~= -1
            delete_block(block);
        end
    end
end
end

function closeAndDeleteLiveModel(model, modelFile)
if bdIsLoaded(model)
    if ~strcmp(get_param(model, 'SimulationStatus'), 'stopped')
        set_param(model, 'SimulationCommand', 'stop');
    end
    close_system(model, 0);
end
if isfile(modelFile)
    delete(modelFile);
end
autosaveFile = [modelFile '.autosave'];
if isfile(autosaveFile)
    delete(autosaveFile);
end
end
