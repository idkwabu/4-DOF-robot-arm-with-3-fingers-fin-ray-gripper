function follow_dume_v3(varargin)
%FOLLOW_DUME_V3 Follow DUM-E state changes in a live Simscape animation.
% Run from MATLAB after changing to this file's folder:
%   follow_dume_v3
% Optional arguments: startGui, stopTime, moveDuration, stateFile.
%
% This is the v3 equivalent of follow_dume_v2.  The live model is rebuilt from
% the clean generated source on every run.  Five arm joints are commanded
% directly from the sampled pose source.  The gripper slider receives a
% continuous position/velocity/acceleration path so the closed linkage does
% not lose its kinematic constraints.

if nargin == 1 && isa(varargin{1}, 'Simulink.MSFcnRunTimeBlock')
    setupPoseSource(varargin{1});
    return
end
if nargin == 1 && (ischar(varargin{1}) || ...
        (isstring(varargin{1}) && isscalar(varargin{1})))
    if strcmp(char(varargin{1}), 'rebuildSource')
        rebuildSourceModelCommand();
        return
    end
end
runFollower(varargin{:});
end

function runFollower(startGui, stopTime, moveDuration, stateFile)
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
addpath(modelFolder);
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
initialPose = readDumePose(stateFile, gripperClosedMeters, gripperOpenMeters);

sourceModelName = 'dum_e_v3_generated';
sourceModelFile = fullfile(modelFolder, [sourceModelName '.slx']);
if bdIsLoaded(sourceModelName) && ...
        ~strcmp(get_param(sourceModelName, 'SimulationStatus'), 'stopped')
    error('Stop the existing %s simulation before starting the DUM-E follower.', ...
        sourceModelName);
end
if ~isfile(sourceModelFile) || ~isCleanSourceModel(sourceModelName, sourceModelFile)
    fprintf('Rebuilding a clean %s from the URDF...\n', sourceModelName);
    rebuildSourceModel(modelFolder, sourceModelFile);
end

liveModelName = 'dum_e_v3_live';
liveModelFile = fullfile(modelFolder, [liveModelName '.slx']);
if bdIsLoaded(liveModelName)
    if ~strcmp(get_param(liveModelName, 'SimulationStatus'), 'stopped')
        error('Stop the existing %s simulation before starting the DUM-E follower.', ...
            liveModelName);
    end
    close_system(liveModelName, 0);
end
copyfile(sourceModelFile, liveModelFile);
liveModelCleanup = onCleanup(@() stopLiveSimulation(liveModelName));
loadLiveModel(liveModelFile);
model = liveModelName;

set_param([model '/MechanismConfiguration'], 'GravityVector', '[0 0 0]');
configureContinuousInputs(model, jointNames, initialPose, stateFile, ...
    gripperClosedMeters, gripperOpenMeters, moveDuration);
set_param(model, ...
    'StopTime', sprintf('%.9g', stopTime), ...
    'EnablePacing', 'on', ...
    'PacingRate', '1', ...
    'MaxStep', '0.01');
save_system(model, liveModelFile);
fprintf('Saved the configured live model: %s\n', liveModelFile);

open_system(model);
fprintf(['Starting one continuous Simscape run; DUM-E poses are sampled from:\n%s\n' ...
    'Gripper 0-100%% maps to the v3 slider range %.1f to %.1f mm.\n' ...
    'The original CAD gripper closures are preserved and the slider drives the linkage.\n' ...
    'Simulation waits between poses and paces each move for visible animation over %.2f simulated seconds.\n' ...
    'Gravity is disabled for stable live pose tracking.\n' ...
    'Press Ctrl+C in MATLAB to stop.\n'], ...
    stateFile, gripperClosedMeters*1000, gripperOpenMeters*1000, moveDuration);
sim(model);
end

function setupPoseSource(block)
block.NumDialogPrms = 5;
sampleTime = block.DialogPrm(2).Data;
if sampleTime <= 0 || block.DialogPrm(3).Data <= 0
    error('follow_dume_v3:InvalidTiming', ...
        'SampleTime and MoveDuration must be positive.');
end
n = numTrackedJoints();
block.NumInputPorts = 0;
block.NumOutputPorts = 3;
for k = 1:3
    block.OutputPort(k).Dimensions = n;
    block.OutputPort(k).DatatypeID = 0;
    block.OutputPort(k).Complexity = 'Real';
end
block.SampleTimes = [sampleTime 0];
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('PostPropagationSetup', @configurePoseDworks);
block.RegBlockMethod('Start', @startPoseSource);
block.RegBlockMethod('Outputs', @outputPose);
block.RegBlockMethod('Update', @updatePoseSource);
end

function n = numTrackedJoints()
n = 6;
end

function configurePoseDworks(block)
block.NumDworks = 2;
for k = 1:2
    block.Dwork(k).Name = sprintf('poseState%d', k);
    block.Dwork(k).Dimensions = 10*numTrackedJoints() + 1;
    block.Dwork(k).DatatypeID = 0;
    block.Dwork(k).Complexity = 'Real';
    block.Dwork(k).UsedAsDiscState = true;
end
end

function startPoseSource(block)
target = readTargetPose(block);
duration = block.DialogPrm(3).Data;
n = numTrackedJoints();
state = struct( ...
    'TargetPose', target, ...
    'StartPose', target, ...
    'StartVelocity', zeros(n,1), ...
    'StartAcceleration', zeros(n,1), ...
    'StartTime', -duration);
state = updateTrajectoryCoefficients(state, duration);
packedState = packPoseState(state);
block.Dwork(1).Data = packedState;
block.Dwork(2).Data = packedState;
end

function outputPose(block)
sampleTime = block.DialogPrm(2).Data;
duration = block.DialogPrm(3).Data;
state = unpackPoseState(block.Dwork(1).Data);
simTime = block.CurrentTime;
[position, velocity, acceleration] = ...
    evaluateTrajectory(state, simTime, duration);

if simTime - state.StartTime >= duration
    target = waitForTargetChange(block, state.TargetPose, sampleTime);
else
    target = readTargetPose(block);
end
if any(abs(target - state.TargetPose) > 1e-10)
    state.StartPose = position;
    state.StartVelocity = velocity;
    state.StartAcceleration = acceleration;
    state.TargetPose = target;
    state.StartTime = simTime;
    state = updateTrajectoryCoefficients(state, duration);
    [position, velocity, acceleration] = ...
        evaluateTrajectory(state, simTime, duration);
end
block.Dwork(2).Data = packPoseState(state);
block.OutputPort(1).Data = position;
block.OutputPort(2).Data = velocity;
block.OutputPort(3).Data = acceleration;
end

function updatePoseSource(block)
block.Dwork(1).Data = block.Dwork(2).Data;
end

function packedState = packPoseState(state)
target = state.TargetPose(:);
startPose = state.StartPose(:);
startVelocity = state.StartVelocity(:);
startAcceleration = state.StartAcceleration(:);
packedState = [target; startPose; startVelocity; startAcceleration; ...
    state.C0(:); state.C1(:); state.C2(:); state.C3(:); ...
    state.C4(:); state.C5(:); state.StartTime];
end

function state = unpackPoseState(packedState)
packedState = packedState(:);
n = (numel(packedState) - 1)/10;
state.TargetPose = packedState(1:n);
state.StartPose = packedState(n + (1:n));
state.StartVelocity = packedState(2*n + (1:n));
state.StartAcceleration = packedState(3*n + (1:n));
state.C0 = packedState(4*n + (1:n));
state.C1 = packedState(5*n + (1:n));
state.C2 = packedState(6*n + (1:n));
state.C3 = packedState(7*n + (1:n));
state.C4 = packedState(8*n + (1:n));
state.C5 = packedState(9*n + (1:n));
state.StartTime = packedState(10*n + 1);
end

function target = waitForTargetChange(block, currentTarget, sampleTime)
while true
    target = readTargetPose(block);
    if any(abs(target - currentTarget) > 1e-10)
        return;
    end
    pause(sampleTime);
end
end

function target = readTargetPose(block)
stateFile = block.DialogPrm(1).Data;
closedMeters = block.DialogPrm(4).Data;
openMeters = block.DialogPrm(5).Data;
target = readDumePose(stateFile, closedMeters, openMeters);
end

function state = updateTrajectoryCoefficients(state, duration)
displacement = state.TargetPose - state.StartPose;
state.C0 = state.StartPose;
state.C1 = duration*state.StartVelocity;
state.C2 = 0.5*duration^2*state.StartAcceleration;
state.C3 = 10*displacement - 6*state.C1 - ...
    1.5*duration^2*state.StartAcceleration;
state.C4 = -15*displacement + 8*state.C1 + ...
    1.5*duration^2*state.StartAcceleration;
state.C5 = 6*displacement - 3*state.C1 - ...
    0.5*duration^2*state.StartAcceleration;
end

function [position, velocity, acceleration] = ...
        evaluateTrajectory(state, simTime, duration)
elapsed = simTime - state.StartTime;
if elapsed >= duration
    position = state.TargetPose;
    velocity = zeros(numel(state.TargetPose),1);
    acceleration = zeros(numel(state.TargetPose),1);
    return;
end
u = max(0, elapsed/duration);
position = state.C0 + state.C1*u + state.C2*u^2 + ...
    state.C3*u^3 + state.C4*u^4 + state.C5*u^5;
velocity = (state.C1 + 2*state.C2*u + 3*state.C3*u^2 + ...
    4*state.C4*u^3 + 5*state.C5*u^4)/duration;
acceleration = (2*state.C2 + 6*state.C3*u + ...
    12*state.C4*u^2 + 20*state.C5*u^3)/duration^2;
end

function configureContinuousInputs( ...
        model, jointNames, initialPose, stateFile, ...
        gripperClosedMeters, gripperOpenMeters, moveDuration)
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

stateFileExpr = [char(39) strrep(stateFile, char(39), [char(39) char(39)]) char(39)];
parameters = sprintf('%s, %.16g, %.16g, %.16g, %.16g', ...
    stateFileExpr, 0.02, moveDuration, gripperClosedMeters, gripperOpenMeters);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [model '/DumeLivePoseSource'], ...
    'FunctionName', 'follow_dume_v3', ...
    'Parameters', parameters, ...
    'Position', [700 80 850 150]);
add_block('simulink/Signal Routing/Demux', [model '/DumeSplitPosition'], ...
    'Outputs', num2str(numel(jointNames)), 'Position', [890 75 895 190]);
add_block('simulink/Signal Routing/Demux', [model '/DumeSplitVelocity'], ...
    'Outputs', num2str(numel(jointNames)), 'Position', [890 215 895 330]);
add_block('simulink/Signal Routing/Demux', [model '/DumeSplitAcceleration'], ...
    'Outputs', num2str(numel(jointNames)), 'Position', [890 355 895 470]);

sourcePorts = get_param([model '/DumeLivePoseSource'], 'PortHandles');
splitNames = {'DumeSplitPosition','DumeSplitVelocity','DumeSplitAcceleration'};
for k = 1:3
    splitPorts = get_param([model '/' splitNames{k}], 'PortHandles');
    add_line(model, sourcePorts.Outport(k), splitPorts.Inport(1), 'autorouting', 'on');
end

q = get_param([model '/DumeSplitPosition'], 'PortHandles');
qd = get_param([model '/DumeSplitVelocity'], 'PortHandles');
qdd = get_param([model '/DumeSplitAcceleration'], 'PortHandles');

% A continuous third-order filter smooths the sampled slider target into
% position/velocity/acceleration.  A sampled-and-held slider command loses the
% closed linkage constraints, but the native Simulink filters are fast.
servoTau = max(0.01, moveDuration/10);
den = sprintf('[%s %s %s 1]', sprintf('%.16g', servoTau^3), ...
    sprintf('%.16g', 3*servoTau^2), sprintf('%.16g', 3*servoTau));
numerators = {'[1]', '[1 0]', '[1 0 0]'};
servoFilterPorts = cell(1,3);
for k = 1:3
    filterPath = [model '/DumeServoFilter' num2str(k)];
    add_block('simulink/Continuous/Transfer Fcn', filterPath, ...
        'Numerator', numerators{k}, ...
        'Denominator', den, ...
        'Position', [1000 610+70*k 1110 645+70*k]);
    servoFilterPorts{k} = get_param(filterPath, 'PortHandles');
    add_line(model, q.Outport(numel(jointNames)), ...
        servoFilterPorts{k}.Inport(1), 'autorouting', 'on');
end

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
    if k == numel(jointNames)
        for f = 1:3
            add_line(model, servoFilterPorts{f}.Outport(1), ...
                converterPorts.Inport(f), 'autorouting', 'on');
        end
    else
        add_line(model, q.Outport(k), converterPorts.Inport(1), 'autorouting', 'on');
        add_line(model, qd.Outport(k), converterPorts.Inport(2), 'autorouting', 'on');
        add_line(model, qdd.Outport(k), converterPorts.Inport(3), 'autorouting', 'on');
    end
    add_line(model, converterPorts.RConn(1), jointPorts.LConn(2), 'autorouting', 'on');
end
set_param(model, 'SimulationCommand', 'update');
end

function pose = readDumePose(stateFile, gripperClosedMeters, gripperOpenMeters)
% The GUI rewrites the state file while the simulation is running.  Retry a
% few times so a transient file-lock or partial write never stops the sim.
lastError = [];
for attempt = 1:5
    try
        state = jsondecode(fileread(stateFile));
        break
    catch exception
        lastError = exception;
        pause(0.01);
    end
end
if ~exist('state', 'var') || isempty(state)
    error('DumePoseV3:ReadFailed', ...
        'Could not read DUM-E pose file %s: %s', stateFile, lastError.message);
end
fields = {'base','joint1','joint2','joint3','gripper_roll','gripper'};
ranges = [-180 180; -90 90; -150 150; -120 120; -180 180; 0 100];
values = zeros(1, numel(fields));
if ~isstruct(state) || ~isscalar(state)
    error('DumePoseV3:InvalidState', 'DUM-E pose must be a JSON object.');
end
for k = 1:numel(fields)
    name = fields{k};
    if ~isfield(state, name) || ~isnumeric(state.(name)) || ...
            ~isscalar(state.(name)) || ~isfinite(state.(name)) || ...
            state.(name) < ranges(k,1) || state.(name) > ranges(k,2)
        error('DumePoseV3:InvalidJoint', ...
            'DUM-E pose has an invalid or out-of-range "%s" value.', name);
    end
    values(k) = state.(name);
end
armPoseDeg = values(1:5) .* [-1 1 1 1 1] + [0 -90 0 0 0];
gripperSlideMeters = gripperClosedMeters + ...
    (values(6)/100)*(gripperOpenMeters-gripperClosedMeters);
pose = [deg2rad(armPoseDeg), gripperSlideMeters].';
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

function addGripperClosures(model, info)
if numel(info.loops) ~= 4
    error('Expected exactly four CAD closure definitions.');
end
closureNames = {info.loops.name};
blocks = get_param(model, 'Blocks');
present = cellfun(@(name) any(strcmp(blocks, name)), closureNames);
if all(present)
    return;
elseif any(present)
    error('The model contains only some of the CAD gripper closure joints.');
end
for k = 1:numel(info.loops)
    closure = info.loops(k);
    frames = {closure.frame_one, closure.frame_two};
    ext = zeros(1,2);
    for side = 1:2
        fr = frames{side};
        sub = [model '/' fr.link];
        tf = [sub '/ClosureTransform'];
        cp = [sub '/ClosurePort'];
        existing = get_param(sub, 'PortHandles');
        add_block('sm_lib/Frames and Transforms/Rigid Transform', tf, ...
            'Position', [430 280 480 330]);
        set_param(tf, ...
            'RotationMethod', 'RotationMatrix', ...
            'RotationMatrix', mat2str(fr.rotation_matrix_in_link_frame, 16), ...
            'TranslationMethod', 'Cartesian', ...
            'TranslationCartesianOffset', ...
            mat2str(fr.translation_m_in_link_frame, 16), ...
            'TranslationCartesianOffsetUnits', 'm');
        add_block([sub '/F'], cp, 'Position', [540 285 570 305]);
        set_param(cp, 'Port', '2', 'Side', 'Right');
        a = get_param([sub '/ReferenceFrame'], 'PortHandles');
        b = get_param(tf, 'PortHandles');
        c = get_param(cp, 'PortHandles');
        add_line(sub, a.RConn(1), b.LConn(1), 'autorouting', 'on');
        add_line(sub, b.RConn(1), c.RConn(1), 'autorouting', 'on');
        p = get_param(sub, 'PortHandles');
        created = setdiff(p.RConn, existing.RConn, 'stable');
        if numel(created) ~= 1
            error('Cannot uniquely identify the new closure port on %s.', sub);
        end
        ext(side) = created(1);
    end
    joint = [model '/' closure.name];
    % Spherical joints keep each linkage end attached to its frame without
    % the redundant axis-alignment constraint of a revolute joint.  This is
    % the same closure strategy used by the v2 gripper fix.
    add_block('sm_lib/Joints/Spherical Joint', joint, ...
        'Position', [1000 120*k 1060 (120*k+45)]);
    jp = get_param(joint, 'PortHandles');
    add_line(model, ext(2), jp.LConn(1), 'autorouting', 'on');
    add_line(model, ext(1), jp.RConn(1), 'autorouting', 'on');
end
set_param(model, 'SimulationCommand', 'update');
end

function rebuildSourceModelCommand()
modelFolder = fileparts(mfilename('fullpath'));
rebuildSourceModel(modelFolder, ...
    fullfile(modelFolder, 'dum_e_v3_generated.slx'));
end

function rebuildSourceModel(modelFolder, sourceModelFile)
name = 'dum_e_v3_generated';
if bdIsLoaded(name)
    if ~strcmp(get_param(name, 'SimulationStatus'), 'stopped')
        error('Stop the existing %s simulation before rebuilding the model.', name);
    end
    close_system(name, 0);
end
if isfile(sourceModelFile)
    delete(sourceModelFile);
end
oldFolder = cd(modelFolder);
folderCleanup = onCleanup(@() cd(oldFolder));
info = jsondecode(fileread(fullfile(modelFolder, 'loop_constraints.json')));
smimport(fullfile(modelFolder, 'dum_e_v3_mk19.urdf'), 'ModelName', name);
model = getfullname(name);
set_param([model '/MechanismConfiguration'], ...
    'UniformGravity', 'Constant', ...
    'GravityVector', '[0 -9.80665 0]', ...
    'GravityVectorUnits', 'm/s^2');
addGripperClosures(model, info);
save_system(model, sourceModelFile);
close_system(model, 0);
fprintf('Rebuilt clean source model: %s\n', sourceModelFile);
end

function tf = isCleanSourceModel(name, modelFile)
tf = false;
loadedHere = false;
if ~bdIsLoaded(name)
    try
        loadLiveModel(modelFile);
        loadedHere = true;
    catch
        return
    end
end
closeCleanup = onCleanup(@() closeLoadedModel(name, loadedHere));
blocks = get_param(name, 'Blocks');
if any(~cellfun(@isempty, regexp(blocks, '^Dume|^dumeUnits_')))
    return
end
closures = {
    'gripper_pos_connector_to_lower_hinge'
    'gripper_neg_connector_to_lower_hinge'
    'gripper_pos_upper_to_jaw_hinge'
    'gripper_neg_upper_to_jaw_hinge'
};
if ~all(cellfun(@(n) any(strcmp(blocks, n)), closures))
    return
end
for k = 1:numel(closures)
    reference = get_param([name '/' closures{k}], 'ReferenceBlock');
    if ~strcmp(strtrim(reference), 'sm_lib/Joints/Spherical Joint')
        return
    end
end
gravity = get_param([name '/MechanismConfiguration'], 'GravityVector');
if ~strcmp(strtrim(gravity), '[0 -9.80665 0]')
    return
end
tf = true;
end

function closeLoadedModel(name, loadedHere)
if loadedHere && bdIsLoaded(name)
    close_system(name, 0);
end
end

function stopLiveSimulation(model)
if bdIsLoaded(model) && ...
        ~strcmp(get_param(model, 'SimulationStatus'), 'stopped')
    set_param(model, 'SimulationCommand', 'stop');
end
end

function loadLiveModel(modelFile)
warningState = warning('off', 'SystemBlock:MATLABSystem:SystemNameNotFound');
warningCleanup = onCleanup(@() warning(warningState));
load_system(modelFile);
end
