function follow_dume_v3_realtime(startGui, stopTime)
%FOLLOW_DUME_V3_REALTIME Run a continuous Simscape simulation driven by DUM-E.
% Run in MATLAB with:
%   cd('D:\Dum-E\MatlabSim\dum_e_v3')
%   follow_dume_v3
% The default run lasts a long time; press Ctrl+C in MATLAB to stop it.

if nargin < 1
    startGui = true;
end
if nargin < 2
    stopTime = 1e6;
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
initialPose = read_dume_pose_v3( ...
    stateFile, gripperClosedMeters, gripperOpenMeters);

modelName = 'dum_e_simscape_generated';
modelFile = fullfile(modelFolder, [modelName '.slx']);
if bdIsLoaded(modelName)
    error('Close the existing %s model before starting the DUM-E follower.', modelName);
elseif isfile(modelFile)
    load_system(modelFile);
    model = modelName;
else
    model = import_dume_simscape();
end
modelCleanup = onCleanup(@() closeImportedModel(model));

set_param([model '/MechanismConfiguration'], 'GravityVector', '[0 0 0]');
configureContinuousInputs(model, jointNames, initialPose, stateFile, ...
    gripperClosedMeters, gripperOpenMeters);
set_param(model, ...
    'StopTime', sprintf('%.9g', stopTime), ...
    'EnablePacing', 'on', ...
    'PacingRate', '1', ...
    'MaxStep', '0.01');

fprintf(['Starting one continuous Simscape run; DUM-E poses are sampled from:\n%s\n' ...
    'Gripper 0-100%% maps to slider travel %.1f to %.1f mm.\n' ...
    'Gravity is disabled for stable live pose tracking. The simulation is paced at real time.\n' ...
    'Press Ctrl+C in MATLAB to stop.\n'], ...
    stateFile, gripperClosedMeters*1000, gripperOpenMeters*1000);
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
        gripperClosedMeters, gripperOpenMeters)
removeOldPoseInputs(model, jointNames);

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
end
set_param(model, 'SimulationCommand', 'update');

for k = 1:numel(jointNames)
    joint = [model '/' jointNames{k}];
    units = 'rad';
    if strcmp(jointNames{k}, 'gripper_slide')
        units = 'm';
    end
    set_param(joint, ...
        'MotionActuationMode', 'InputMotion', ...
        'TorqueActuationMode', 'ComputedTorque', ...
        'PositionTargetSpecify', 'on', ...
        'PositionTargetPriority', 'High', ...
        'PositionTargetValue', sprintf('%.16g', initialPose(k)), ...
        'PositionTargetValueUnits', units);
end
set_param(model, 'SimulationCommand', 'update');

add_block('simulink/User-Defined Functions/MATLAB System', ...
    [model '/DumeLivePoseSource'], ...
    'Position', [700 80 850 150]);
set_param([model '/DumeLivePoseSource'], ...
    'System', 'DumePoseSourceV3', ...
    'SimulateUsing', 'Interpreted execution', ...
    'StateFile', stateFile, ...
    'SampleTime', '0.02', ...
    'MoveDuration', '0.8', ...
    'GripperClosedMeters', sprintf('%.16g', gripperClosedMeters), ...
    'GripperOpenMeters', sprintf('%.16g', gripperOpenMeters));

add_block('simulink/Sources/Clock', [model '/DumeLiveClock'], ...
    'Position', [570 90 600 120]);
add_block('simulink/Signal Routing/Demux', [model '/DumeSplitPosition'], ...
    'Outputs', '6', 'Position', [890 75 895 190]);
add_block('simulink/Signal Routing/Demux', [model '/DumeSplitVelocity'], ...
    'Outputs', '6', 'Position', [890 215 895 330]);
add_block('simulink/Signal Routing/Demux', [model '/DumeSplitAcceleration'], ...
    'Outputs', '6', 'Position', [890 355 895 470]);
add_block('simulink/Discrete/Unit Delay', [model '/DumeHoldPosition'], ...
    'Position', [925 90 965 120]);
add_block('simulink/Discrete/Unit Delay', [model '/DumeHoldVelocity'], ...
    'Position', [925 230 965 260]);
add_block('simulink/Discrete/Unit Delay', [model '/DumeHoldAcceleration'], ...
    'Position', [925 370 965 400]);
set_param([model '/DumeHoldPosition'], ...
    'InitialCondition', mat2str(initialPose, 16));
set_param([model '/DumeHoldVelocity'], 'InitialCondition', 'zeros(6,1)');
set_param([model '/DumeHoldAcceleration'], 'InitialCondition', 'zeros(6,1)');

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
    converter = [model '/dumeUnits_' jointName];
    units = 'rad';
    if strcmp(jointName, 'gripper_slide')
        units = 'm';
    end
    add_block('nesl_utility/Simulink-PS Converter', converter, ...
        'Unit', units, ...
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
        ['dumeUnits_' jointNames{k}]};
end

blocks = get_param(model, 'Blocks');
for k = 1:numel(oldNames)
    if any(strcmp(blocks, oldNames{k}))
        delete_block([model '/' oldNames{k}]);
    end
end
end

function closeImportedModel(model)
if bdIsLoaded(model)
    close_system(model, 0);
end
end
