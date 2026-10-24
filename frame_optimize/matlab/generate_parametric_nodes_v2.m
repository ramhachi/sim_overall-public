function [nodes, fbh_nodes, suspension_mount_nodes, inner_bracket_nodes, X_fh, X_mh, fbh_idx, susp_idx, rear_susp_idx, meta] = generate_parametric_nodes_v2(geom, suspension_file, design, config)
% GENERATE_PARAMETRIC_NODES_V2 Build a V2 parametric FSAE frame node set.
%
% V2 makes FBH/FH/MH/MH brace/rear trapezoid dimensions design variables
% and requires differential mount input before rear free-region generation.

    if nargin < 4 || isempty(config)
        config = struct();
    end
    if ~isfield(design, 'schema_version') || ~strcmp(design.schema_version, 'frame_design/v2')
        error('generate_parametric_nodes_v2:invalidDesign', ...
            'generate_parametric_nodes_v2 requires a frame_design/v2 decoded design.');
    end
    if ~isfield(config, 'diff_mount_points') || isempty(config.diff_mount_points)
        error('generate_parametric_nodes_v2:MissingDiffMountConfig', ...
            'V2 requires config.diff_mount_points; do not run with guessed differential geometry.');
    end

    g = design.geometry;
    L = geom.dimensions.wheelbase_m;
    z_floor = 0.050;
    adjustments = struct();

    [fbh_nodes_susp, suspension_mount_nodes_susp, upright_nodes_susp, has_suspension] = ...
        load_suspension_hardpoints(suspension_file, 0.500 - g.fbh_x_shift_m, ...
        g.fbh_z_offset_m, 2 * g.fbh_half_width_m, g.fbh_height_m, false);

    if ~has_suspension
        error('generate_parametric_nodes_v2:MissingSuspension', ...
            'Suspension data must be present for V2 frame optimization.');
    end

    [rear_mount_nodes_raw, rear_upright_nodes, has_rear_suspension] = ...
        load_rear_suspension_hardpoints(suspension_file, L, false);

    fbh_nodes = fbh_nodes_susp;
    suspension_mount_nodes = suspension_mount_nodes_susp;
    X_fh = max(suspension_mount_nodes(:, 1)) + 0.150;
    X_mh = X_fh + g.mh_x_offset_from_fh_m;
    X_si = X_fh + (X_mh - X_fh) * g.si_x_ratio;

    inner_bracket_nodes = makeInnerBracketNodes(suspension_mount_nodes, upright_nodes_susp);

    si_lower_z = z_floor + 0.050;
    si_upper_z = si_lower_z + g.si_upper_delta_m;

    mh_mid_z = max(g.fh_top_z_m - 0.200, si_upper_z + 0.050);
    requested_shoulder_z = g.shoulder_bar_z_m;
    shoulder_z = min(max(requested_shoulder_z, mh_mid_z + 0.030), g.mh_top_z_m - 0.040);
    if abs(shoulder_z - requested_shoulder_z) > 1e-9
        adjustments.shoulder_bar_z_m = struct( ...
            'requested', requested_shoulder_z, 'applied', shoulder_z);
    end
    requested_brace_attach_z = g.mh_brace_attach_z_m;
    brace_attach_z = min(max(requested_brace_attach_z, shoulder_z), g.mh_top_z_m);
    if abs(brace_attach_z - requested_brace_attach_z) > 1e-9
        adjustments.mh_brace_attach_z_m = struct( ...
            'requested', requested_brace_attach_z, 'applied', brace_attach_z);
    end
    brace_attach_half_width = interpolateMainHoopHalfWidth( ...
        brace_attach_z, shoulder_z, g.mh_top_z_m, g.mh_shoulder_half_width_m);

    X_brace_end = X_mh + g.mh_brace_end_x_offset_m;
    X_rear_free = X_brace_end + g.rear_free_x_offset_m;

    diff_mount_points = normalizePointMatrix(config.diff_mount_points);
    [diff_mount_points, diff_shift] = enforceAftOfBraceEnd(diff_mount_points, X_brace_end);
    if diff_shift > 0
        adjustments.diff_mount_points_shift_m = diff_shift;
    end

    requested_rear_free_x = X_rear_free;
    if has_rear_suspension
        X_rear_free = max(X_rear_free, max(rear_mount_nodes_raw(:, 1)) + 0.050);
        requested_rear_half_width = g.rear_free_half_width_m;
        g.rear_free_half_width_m = max(requested_rear_half_width, max(abs(rear_mount_nodes_raw(:, 2))) * 0.80);
        if abs(g.rear_free_half_width_m - requested_rear_half_width) > 1e-9
            adjustments.rear_free_half_width_m = struct( ...
                'requested', requested_rear_half_width, 'applied', g.rear_free_half_width_m);
        end
        requested_rear_upper_z = g.rear_free_upper_z_m;
        g.rear_free_upper_z_m = max(requested_rear_upper_z, max(rear_mount_nodes_raw(:, 3)) + 0.100);
        if abs(g.rear_free_upper_z_m - requested_rear_upper_z) > 1e-9
            adjustments.rear_free_upper_z_m = struct( ...
                'requested', requested_rear_upper_z, 'applied', g.rear_free_upper_z_m);
        end
    end
    X_rear_free = max(X_rear_free, max(diff_mount_points(:, 1)) + 0.050);
    if abs(X_rear_free - requested_rear_free_x) > 1e-9
        adjustments.rear_free_x_m = struct( ...
            'requested', requested_rear_free_x, 'applied', X_rear_free);
    end

    base_nodes = [
        X_fh, -g.fh_half_width_m, g.fh_lower_z_m;
        X_fh,  g.fh_half_width_m, g.fh_lower_z_m;
        X_fh, -g.fh_half_width_m, g.fh_top_z_m;
        X_fh,  g.fh_half_width_m, g.fh_top_z_m;

        X_si, -g.si_half_width_m, si_lower_z;
        X_si,  g.si_half_width_m, si_lower_z;
        X_si, -g.si_half_width_m, si_upper_z;
        X_si,  g.si_half_width_m, si_upper_z;

        X_mh, -g.mh_base_half_width_m, z_floor;
        X_mh,  g.mh_base_half_width_m, z_floor;
        X_mh, -g.mh_mid_half_width_m, mh_mid_z;
        X_mh,  g.mh_mid_half_width_m, mh_mid_z;
        X_mh, -g.mh_shoulder_half_width_m, shoulder_z;
        X_mh,  g.mh_shoulder_half_width_m, shoulder_z;
        X_mh,  0.0, g.mh_top_z_m;

        X_mh, -brace_attach_half_width, brace_attach_z;
        X_mh,  brace_attach_half_width, brace_attach_z;

        X_brace_end, -g.mh_brace_end_top_half_width_m, g.mh_brace_end_upper_z_m;
        X_brace_end,  g.mh_brace_end_top_half_width_m, g.mh_brace_end_upper_z_m;
        X_brace_end, -g.mh_brace_end_bottom_half_width_m, g.mh_brace_end_lower_z_m;
        X_brace_end,  g.mh_brace_end_bottom_half_width_m, g.mh_brace_end_lower_z_m;

        X_rear_free, -g.rear_free_half_width_m, g.rear_free_upper_z_m;
        X_rear_free,  g.rear_free_half_width_m, g.rear_free_upper_z_m;
        X_rear_free, -g.rear_free_half_width_m, z_floor;
        X_rear_free,  g.rear_free_half_width_m, z_floor;
    ];

    nodes = base_nodes;
    rear_mid_idx = [];
    if design.node_toggles(2) ~= 0 && X_rear_free - X_brace_end > 0.20
        X_rear_mid = X_brace_end + (X_rear_free - X_brace_end) * g.rear_mid_x_ratio;
        z_mid_lower = z_floor + (g.rear_free_upper_z_m - z_floor) * 0.35 * g.rear_mid_z_ratio;
        z_mid_upper = min(g.rear_free_upper_z_m, z_mid_lower + 0.30);
        w_mid = 0.5 * (g.mh_brace_end_bottom_half_width_m + g.rear_free_half_width_m);
        rear_mid_idx = size(nodes, 1) + (1:4);
        nodes = [nodes; ...
            X_rear_mid, -w_mid, z_mid_upper; ...
            X_rear_mid,  w_mid, z_mid_upper; ...
            X_rear_mid, -w_mid, z_mid_lower; ...
            X_rear_mid,  w_mid, z_mid_lower];
    end

    fbh_start = size(nodes, 1) + 1;
    nodes = [nodes; fbh_nodes];
    fbh_idx = fbh_start:size(nodes, 1);

    susp_start = size(nodes, 1) + 1;
    nodes = [nodes; suspension_mount_nodes];
    susp_idx = susp_start:size(nodes, 1);

    inner_start = size(nodes, 1) + 1;
    nodes = [nodes; inner_bracket_nodes];
    inner_idx = inner_start:size(nodes, 1);

    rear_susp_idx = [];
    rear_inner_idx = [];
    if has_rear_suspension
        rear_susp_start = size(nodes, 1) + 1;
        nodes = [nodes; rear_mount_nodes_raw];
        rear_susp_idx = rear_susp_start:size(nodes, 1);

        rear_inner_nodes = makeInnerBracketNodes(rear_mount_nodes_raw, rear_upright_nodes);
        rear_inner_start = size(nodes, 1) + 1;
        nodes = [nodes; rear_inner_nodes];
        rear_inner_idx = rear_inner_start:size(nodes, 1);
    end

    diff_start = size(nodes, 1) + 1;
    nodes = [nodes; diff_mount_points];
    diff_mount_idx = diff_start:size(nodes, 1);

    meta = struct();
    meta.schema_version = 'frame_design/v2';
    meta.group_names = design.group_names;
    meta.front_hoop_idx = 1:4;
    meta.side_impact_idx = 5:8;
    meta.main_hoop_idx = 9:17;
    meta.main_hoop_brace_attach_idx = 16:17;
    meta.main_hoop_top_idx = 15;  % [X_mh, 0, mh_top_z] row above (rollover load point)
    meta.brace_trapezoid_idx = 18:21;
    meta.rear_free_idx = 22:25;
    meta.rear_mid_idx = rear_mid_idx;
    meta.fbh_idx = fbh_idx;
    meta.susp_idx = susp_idx;
    meta.inner_bracket_idx = inner_idx;
    meta.rear_susp_idx = rear_susp_idx;
    meta.rear_inner_bracket_idx = rear_inner_idx;
    meta.diff_mount_idx = diff_mount_idx;
    meta.X_fh = X_fh;
    meta.X_mh = X_mh;
    meta.X_brace_end = X_brace_end;
    meta.X_rear_free = X_rear_free;
    meta.side_impact_lower_z = si_lower_z;
    meta.side_impact_upper_z = si_upper_z;
    meta.mh_top_z = g.mh_top_z_m;
    meta.mh_brace_attach_z = brace_attach_z;
    meta.design_adjustments = adjustments;
end

function inner_nodes = makeInnerBracketNodes(mount_nodes, upright_nodes)
    L_bracket = 0.050;
    map = [1, 2, 3, 4, 1, 2, 3, 4];
    n = size(mount_nodes, 1);
    inner_nodes = zeros(n, 3);
    for i = 1:n
        upright_idx = map(min(i, numel(map)));
        upright_pt = upright_nodes(upright_idx, :);
        mount_pt = mount_nodes(i, :);
        arm_vec = mount_pt - upright_pt;
        arm_norm = norm(arm_vec);
        if arm_norm < eps
            unit_vec = [1, 0, 0];
        else
            unit_vec = arm_vec / arm_norm;
        end
        inner_nodes(i, :) = mount_pt + unit_vec * L_bracket;
    end
end

function half_width = interpolateMainHoopHalfWidth(z, shoulder_z, top_z, shoulder_half_width)
    if top_z <= shoulder_z + eps
        half_width = shoulder_half_width;
        return;
    end
    ratio = min(max((z - shoulder_z) / (top_z - shoulder_z), 0), 1);
    half_width = shoulder_half_width * (1 - ratio);
end

function points = normalizePointMatrix(points)
    if size(points, 2) ~= 3 && size(points, 1) == 3
        points = points';
    end
    if size(points, 2) ~= 3 || size(points, 1) < 2
        error('generate_parametric_nodes_v2:invalidDiffMountPoints', ...
            'config.diff_mount_points must be an Nx3 matrix with at least two points.');
    end
end

function [points, shift] = enforceAftOfBraceEnd(points, x_brace_end)
    min_allowed_x = x_brace_end + 0.020;
    shift = 0;
    if any(points(:, 1) < min_allowed_x)
        shift = min_allowed_x - min(points(:, 1));
        points(:, 1) = points(:, 1) + shift;
    end
end
