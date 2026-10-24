function lc = import_frame_load_cases(load_cases_file, suspension_file)
% IMPORT_FRAME_LOAD_CASES Read chassis mount loads (e.g. a 14DOF envelope) for the frame FEM (#49).
%
%   lc = import_frame_load_cases(load_cases_file, suspension_file)
%
% File schema "frame_load_cases/v1" (forces on the CHASSIS, SAE J670 vehicle axes:
% X forward, Y right, Z down; N and N*m):
%   {
%     "schema": "frame_load_cases/v1",
%     "source": "14dof_run_<id>",            % required, kept in every case
%     "cases": [
%       {"name": "autocross_peak_lat",
%        "mount_loads": [ {"corner": "FL", "hardpoint": "lower_arm_chassis_fore",
%                          "F": [fx, fy, fz], "M": [mx, my, mz]},       % M optional
%                         {"diff_mount": 1, "F": [...], "M": [...]} ],
%        "arm_loads":   [ {"corner": "RR", "arm": "upper", "F": [fx, fy, fz]} ] }
%     ]
%   }
%   corner: FL/FR/RL/RR; hardpoint: {lower,upper}_arm_chassis_{fore,aft}.
%   arm_loads: force the A-arm carries at its ball joint toward the chassis; it is split
%   along the two legs (ball joint -> fore/aft pickup) by least squares. The out-of-plane
%   residual (arm bending, not seen by the frame) is reported in .arm_residual_N.
%
% Output (frame CS: X rearward, Y right, Z up):
%   lc.source, lc.cases(k).name, lc.cases(k).mount_loads = [axle idx Fx Fy Fz Mx My Mz]
%   axle: 1 = front suspension mount, 2 = rear suspension mount, 3 = diff mount;
%   idx follows the generator order (1/2 lower fore L/R, 3/4 upper fore, 5/6 lower aft,
%   7/8 upper aft) or the diff mount number.

    persistent cache_key cache_value
    key = [char(load_cases_file) '|' char(suspension_file)];
    if ~isempty(cache_key) && strcmp(cache_key, key)
        lc = cache_value;
        return;
    end

    raw = jsondecode(fileread(load_cases_file));
    if ~isfield(raw, 'schema') || ~strcmp(raw.schema, 'frame_load_cases/v1')
        error('import_frame_load_cases:Schema', 'Expected schema frame_load_cases/v1 in %s', load_cases_file);
    end
    if ~isfield(raw, 'source') || isempty(raw.source)
        error('import_frame_load_cases:MissingSource', 'Load case file must state its "source" (e.g. 14dof_run_<id>).');
    end

    artifact = load_vehicle_artifact(suspension_file);
    corners = artifact.params.suspension.kc.corners;
    hp_names = {'lower_arm_chassis_fore', 'upper_arm_chassis_fore', 'lower_arm_chassis_aft', 'upper_arm_chassis_aft'};
    sae_to_frame = diag([-1, 1, -1]);  % 180 deg about Y: proper rotation, same for F and M

    raw_cases = raw.cases;
    if ~iscell(raw_cases)
        raw_cases = num2cell(raw_cases);
    end
    lc = struct('source', raw.source, 'cases', struct('name', {}, 'mount_loads', {}, 'arm_residual_N', {}));
    for k = 1:numel(raw_cases)
        c = raw_cases{k};
        rows = zeros(0, 8);
        residual = 0;
        for m = listOf(c, 'mount_loads')
            ml = m{1};
            F = sae_to_frame * ml.F(:);
            M = zeros(3, 1);
            if isfield(ml, 'M')
                M = sae_to_frame * ml.M(:);
            end
            if isfield(ml, 'diff_mount')
                rows(end+1, :) = [3, ml.diff_mount, F', M']; %#ok<AGROW>
            else
                [axle, idx] = mountIndex(ml.corner, ml.hardpoint);
                rows(end+1, :) = [axle, idx, F', M']; %#ok<AGROW>
            end
        end
        for a = listOf(c, 'arm_loads')
            al = a{1};
            hp = corners.([upper(al.corner(1)) 'L']).hardpoints;  % right side mirrors left
            side = 1 - 2 * any(upper(al.corner(2)) == 'L');       % -1 left, +1 right
            to_frame = @(p) [-p(1); side * abs(p(2)); -p(3)];
            ball = to_frame(hp.([al.arm '_ball_joint']));
            u_fore = to_frame(hp.([al.arm '_arm_chassis_fore'])) - ball;
            u_aft = to_frame(hp.([al.arm '_arm_chassis_aft'])) - ball;
            U = [u_fore / norm(u_fore), u_aft / norm(u_aft)];
            F = sae_to_frame * al.F(:);
            leg = U \ F;  % axial leg forces (+ = toward the chassis pickup)
            residual = max(residual, norm(F - U * leg));
            [axle, idx_fore] = mountIndex(al.corner, [al.arm '_arm_chassis_fore']);
            [~, idx_aft] = mountIndex(al.corner, [al.arm '_arm_chassis_aft']);
            rows(end+1, :) = [axle, idx_fore, (leg(1) * U(:, 1))', 0, 0, 0]; %#ok<AGROW>
            rows(end+1, :) = [axle, idx_aft, (leg(2) * U(:, 2))', 0, 0, 0]; %#ok<AGROW>
        end
        lc.cases(k).name = c.name;
        lc.cases(k).mount_loads = rows;
        lc.cases(k).arm_residual_N = residual;
    end

    cache_key = key;
    cache_value = lc;

    function [axle, idx] = mountIndex(corner, hardpoint)
        corner = upper(char(corner));
        axle = 1 + (corner(1) == 'R');
        h = find(strcmp(hp_names, hardpoint), 1);
        if isempty(h) || ~any(corner(2) == 'LR')
            error('import_frame_load_cases:UnknownMount', 'Unknown mount %s/%s', corner, hardpoint);
        end
        idx = 2 * (h - 1) + 1 + (corner(2) == 'R');
    end
end

function items = listOf(s, field)
    items = {};
    if isfield(s, field) && ~isempty(s.(field))
        items = s.(field);
        if ~iscell(items)
            items = num2cell(items);
        end
        items = reshape(items, 1, []);
    end
end
