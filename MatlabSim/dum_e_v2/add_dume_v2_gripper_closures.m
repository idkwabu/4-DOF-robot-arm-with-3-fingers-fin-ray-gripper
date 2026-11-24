function add_dume_v2_gripper_closures(model, info, updateModel)
%ADD_DUME_V2_GRIPPER_CLOSURES Restore the three CAD gripper loop joints.

if nargin < 2
    folder = fileparts(mfilename('fullpath'));
    info = jsondecode(fileread(fullfile(folder, 'loop_constraints.json')));
end
if nargin < 3
    updateModel = true;
end
if numel(info.loops) ~= 3
    error('Expected exactly three CAD closure definitions.');
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
        if ~isempty(existing.RConn)
            error('Unexpected existing right-side port on %s.', sub);
        end
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
        ext(side) = p.RConn(1);
    end
    joint = [model '/' closure.name];
    add_block('sm_lib/Joints/Revolute Joint', joint, ...
        'Position', [1000 120*k 1060 (120*k+45)]);
    jp = get_param(joint, 'PortHandles');
    add_line(model, ext(2), jp.LConn(1), 'autorouting', 'on');
    add_line(model, ext(1), jp.RConn(1), 'autorouting', 'on');
end
if updateModel
    set_param(model, 'SimulationCommand', 'update');
end
end
