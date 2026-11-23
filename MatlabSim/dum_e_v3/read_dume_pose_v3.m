function [pose, state] = read_dume_pose_v3(stateFile, gripperClosedMeters, gripperOpenMeters)
%READ_DUME_POSE_V3 Read and map one validated DUM-E state to v3 joint units.

if nargin < 2
    gripperClosedMeters = -0.009;
end
if nargin < 3
    gripperOpenMeters = 0.010;
end
validateattributes(gripperClosedMeters, {'numeric'}, {'scalar','real','finite'});
validateattributes(gripperOpenMeters, {'numeric'}, {'scalar','real','finite'});
if gripperClosedMeters == gripperOpenMeters
    error('DumePoseV3:InvalidGripperRange', ...
        'The closed and open gripper positions must differ.');
end

try
    state = jsondecode(fileread(stateFile));
catch exception
    error('DumePoseV3:ReadFailed', ...
        'Could not read DUM-E pose file %s: %s', stateFile, exception.message);
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
