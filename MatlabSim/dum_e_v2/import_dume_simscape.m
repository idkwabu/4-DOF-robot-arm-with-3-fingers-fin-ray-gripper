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
if numel(info.loops) ~= 3
    error('Expected exactly three CAD closure definitions.');
end
model = smimport(fullfile(folder, 'dum_e_v2_mk16.urdf'), 'ModelName', name);
model = getfullname(model);
% The Fusion source is Y-up. Configure standard gravity in the -Y direction.
mc = [model '/MechanismConfiguration'];
set_param(mc, 'UniformGravity', 'Constant', ...
    'GravityVector', '[0 -9.80665 0]', 'GravityVectorUnits', 'm/s^2');
for k = 1:numel(info.loops)
    closure = info.loops(k);
    frames = {closure.frame_one, closure.frame_two};
    ext = zeros(1,2);
    for side = 1:2
        fr = frames{side};
        sub = [model '/' fr.link];
        tf = [sub '/ClosureTransform'];
        cp = [sub '/ClosurePort'];
        existing = get_param(sub,'PortHandles');
        if ~isempty(existing.RConn)
            error('Unexpected existing right-side port on %s.', sub);
        end
        add_block('sm_lib/Frames and Transforms/Rigid Transform', tf, 'Position', [430 280 480 330]);
        set_param(tf, 'RotationMethod', 'RotationMatrix', ...
            'RotationMatrix', mat2str(fr.rotation_matrix_in_link_frame,16), ...
            'TranslationMethod', 'Cartesian', ...
            'TranslationCartesianOffset', mat2str(fr.translation_m_in_link_frame,16), ...
            'TranslationCartesianOffsetUnits', 'm');
        add_block([sub '/F'], cp, 'Position', [540 285 570 305]);
        set_param(cp, 'Port', '2', 'Side', 'Right');
        a = get_param([sub '/ReferenceFrame'],'PortHandles');
        b = get_param(tf,'PortHandles');
        c = get_param(cp,'PortHandles');
        add_line(sub, a.RConn(1), b.LConn(1), 'autorouting', 'on');
        add_line(sub, b.RConn(1), c.RConn(1), 'autorouting', 'on');
        p = get_param(sub,'PortHandles');
        ext(side) = p.RConn(1);
    end
    joint = [model '/' closure.name];
    add_block('sm_lib/Joints/Revolute Joint', joint, ...
        'Position', [1000 120*k 1060 (120*k+45)]);
    jp = get_param(joint,'PortHandles');
    add_line(model, ext(2), jp.LConn(1), 'autorouting', 'on');
    add_line(model, ext(1), jp.RConn(1), 'autorouting', 'on');
end
set_param(model,'SimulationCommand','update');
fprintf(['Imported %s and restored 3 gripper closures.\n' ...
    'IMPORTANT: Supply verified mass/inertia and actuator data before dynamic use.\n' ...
    'The source is Y-up. Gravity is set to [0 -9.80665 0] m/s^2.\n'], model);
end
