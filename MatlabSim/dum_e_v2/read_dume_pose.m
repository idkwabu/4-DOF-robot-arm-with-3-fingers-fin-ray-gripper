function [pose, state] = read_dume_pose(stateFile, gripperClosedDeg, gripperOpenDeg)
%READ_DUME_POSE Read and map one validated Dum-E state to Simscape radians.

if nargin < 2
    gripperClosedDeg = -120;
end
if nargin < 3
    gripperOpenDeg = 65;
end

try
    state = jsondecode(fileread(stateFile));
catch exception
    error('DumePose:ReadFailed', ...
        'Could not read DUM-E pose file %s: %s', stateFile, exception.message);
end

fields = {'base','joint1','joint2','joint3','gripper_roll','gripper'};
ranges = [-180 180; -90 90; -150 150; -120 120; -180 180; 0 100];
values = zeros(1, numel(fields));
if ~isstruct(state) || ~isscalar(state)
    error('DumePose:InvalidState', 'DUM-E pose must be a JSON object.');
end
for k = 1:numel(fields)
    name = fields{k};
    if ~isfield(state, name) || ~isnumeric(state.(name)) || ...
            ~isscalar(state.(name)) || ~isfinite(state.(name)) || ...
            state.(name) < ranges(k,1) || state.(name) > ranges(k,2)
        error('DumePose:InvalidJoint', ...
            'DUM-E pose has an invalid or out-of-range "%s" value.', name);
    end
    values(k) = state.(name);
end

armPoseDeg = values(1:5) .* [-1 1 1 1 1] + [0 -90 0 0 0];
gripperServoDeg = gripperClosedDeg + ...
    (values(6)/100)*(gripperOpenDeg-gripperClosedDeg);
pose = deg2rad([armPoseDeg gripperServoDeg]).';
end
