function [pass, penalty, report] = check_frame_geometry_constraints_v2(nodes, meta, design, config, elements_geom)
% CHECK_FRAME_GEOMETRY_CONSTRAINTS_V2 Cheap regulation/physics checks before FEM.

    if nargin < 5
        elements_geom = struct();
    end
    if nargin < 4 || isempty(config)
        config = struct();
    end

    pass = true;
    penalty = 0;
    report = struct();
    report.schema_version = 'frame_geometry_check/v2';
    report.violations = {};
    report.metrics = struct();

    g = design.geometry;

    mh_inner_width_m = 2 * g.mh_base_half_width_m;
    report.metrics.main_hoop_centerline_width_m = mh_inner_width_m;
    addViolationIf(mh_inner_width_m < 0.380, 'main_hoop_width', ...
        sprintf('Main hoop centerline width %.3f m is below 0.380 m.', mh_inner_width_m), 1e8);

    fh_tilt_deg = hoopTiltDeg(nodes(1, :), nodes(3, :));
    mh_tilt_deg = max([ ...
        hoopTiltDeg(nodes(9, :), nodes(11, :)), ...
        hoopTiltDeg(nodes(11, :), nodes(13, :)), ...
        hoopTiltDeg(nodes(13, :), nodes(16, :))]);
    report.metrics.front_hoop_tilt_deg = fh_tilt_deg;
    report.metrics.main_hoop_tilt_deg = mh_tilt_deg;
    addViolationIf(fh_tilt_deg > 20.0, 'front_hoop_tilt', ...
        sprintf('Front hoop tilt %.1f deg exceeds 20 deg.', fh_tilt_deg), 1e8);
    addViolationIf(mh_tilt_deg > 10.0, 'main_hoop_tilt', ...
        sprintf('Main hoop tilt %.1f deg exceeds 10 deg.', mh_tilt_deg), 1e8);

    side_impact_delta_m = meta.side_impact_upper_z - meta.side_impact_lower_z;
    report.metrics.side_impact_upper_delta_m = side_impact_delta_m;
    addViolationIf(side_impact_delta_m < 0.290 || side_impact_delta_m > 0.345, 'side_impact_height_band', ...
        sprintf('Upper side impact delta %.3f m is outside 0.290-0.345 m.', side_impact_delta_m), 1e8);

    brace_attach_drop_m = meta.mh_top_z - meta.mh_brace_attach_z;
    report.metrics.mh_brace_attach_drop_m = brace_attach_drop_m;
    addViolationIf(brace_attach_drop_m < -1e-6 || brace_attach_drop_m > 0.160, 'mh_brace_attach_height', ...
        sprintf('Main hoop brace attachment drop %.3f m is outside 0-0.160 m.', brace_attach_drop_m), 1e8);

    left_angle = braceAngleFromVerticalDeg(nodes(16, :), nodes(18, :));
    right_angle = braceAngleFromVerticalDeg(nodes(17, :), nodes(19, :));
    brace_angle_deg = min(left_angle, right_angle);
    report.metrics.mh_brace_angle_deg = brace_angle_deg;
    addViolationIf(brace_angle_deg < 30.0, 'mh_brace_angle', ...
        sprintf('Main hoop brace side-view angle %.1f deg is below 30 deg.', brace_angle_deg), 1e8);

    addViolationIf(g.mh_brace_end_top_half_width_m <= 0 || g.mh_brace_end_bottom_half_width_m <= 0, ...
        'brace_trapezoid_width', 'Brace end trapezoid widths must be positive.', 1e8);
    addViolationIf(g.mh_brace_end_upper_z_m <= g.mh_brace_end_lower_z_m, ...
        'brace_trapezoid_height', 'Brace end trapezoid upper node must be above lower node.', 1e8);
    addViolationIf(meta.X_rear_free <= meta.X_brace_end, 'rear_free_region_order', ...
        'Rear free region must be aft of the mandatory brace trapezoid.', 1e8);

    if ~isfield(config, 'diff_mount_points') || isempty(config.diff_mount_points)
        addViolationIf(true, 'missing_diff_mount_points', ...
            'V2 requires diff_mount_points; guessed differential geometry is not allowed.', 1e8);
    elseif isfield(meta, 'diff_mount_idx') && ~isempty(meta.diff_mount_idx)
        min_diff_x = min(nodes(meta.diff_mount_idx, 1));
        report.metrics.min_diff_mount_x_m = min_diff_x;
        addViolationIf(min_diff_x < meta.X_brace_end - 1e-6, 'diff_mount_before_trapezoid', ...
            'Differential mount points must be in the rear free region aft of the trapezoid.', 1e8);
    end

    cockpit_width_m = 2 * min(g.fh_half_width_m, g.mh_base_half_width_m);
    cockpit_length_m = meta.X_mh - meta.X_fh;
    report.metrics.cockpit_width_proxy_m = cockpit_width_m;
    report.metrics.cockpit_length_proxy_m = cockpit_length_m;
    addViolationIf(cockpit_width_m < 0.250, 'cockpit_width_proxy', ...
        sprintf('Cockpit width proxy %.3f m is below 0.250 m.', cockpit_width_m), 1e8);
    addViolationIf(cockpit_length_m < 0.550, 'driver_template_proxy', ...
        sprintf('FH-MH spacing %.3f m is below the V2 driver-template proxy.', cockpit_length_m), 1e8);

    if isfield(elements_geom, 'connectivity') && ~isempty(elements_geom.connectivity)
        deg = accumarray(elements_geom.connectivity(:), 1, [size(nodes, 1), 1], @sum, 0);
        excluded = [meta.susp_idx(:); meta.rear_susp_idx(:)];
        frame_nodes = setdiff((1:size(nodes, 1))', excluded);
        isolated = frame_nodes(deg(frame_nodes) < 2);
        report.metrics.min_frame_node_degree = min(deg(frame_nodes));
        if ~isempty(isolated)
            addViolationIf(true, 'isolated_frame_nodes', ...
                sprintf('Frame nodes with degree < 2: %s', mat2str(reshape(isolated, 1, []))), 1e8);
        end

        max_pipe_len = getConfigDefault(config, 'max_rear_free_pipe_length_m', 1.20);
        edge_x = reshape(nodes(elements_geom.connectivity(:), 1), size(elements_geom.connectivity));
        rear_edges = elements_geom.connectivity(all(edge_x >= meta.X_brace_end - 1e-6, 2), :);
        if ~isempty(rear_edges)
            lengths = vecnorm(nodes(rear_edges(:, 1), :) - nodes(rear_edges(:, 2), :), 2, 2);
            report.metrics.max_rear_pipe_length_m = max(lengths);
            addViolationIf(any(lengths > max_pipe_len + 1e-6), 'rear_pipe_length', ...
                sprintf('Rear free-region pipe length exceeds %.3f m.', max_pipe_len), 1e8);
        end
    end

    pass = isempty(report.violations);

    function addViolationIf(condition, code, message, value)
        if ~condition
            return;
        end
        pass = false;
        penalty = penalty + value;
        report.violations{end + 1} = struct('code', code, 'message', message, 'penalty', value); %#ok<AGROW>
    end
end

function angle_deg = hoopTiltDeg(lower_pt, upper_pt)
    dz = abs(upper_pt(3) - lower_pt(3));
    dx = abs(upper_pt(1) - lower_pt(1));
    if dz < eps
        angle_deg = 90.0;
    else
        angle_deg = atan2d(dx, dz);
    end
end

function angle_deg = braceAngleFromVerticalDeg(attach_pt, end_pt)
    dx = abs(end_pt(1) - attach_pt(1));
    dz = abs(end_pt(3) - attach_pt(3));
    angle_deg = atan2d(dx, max(dz, eps));
end

function value = getConfigDefault(config, field_name, default_value)
    value = default_value;
    if isstruct(config) && isfield(config, field_name) && ~isempty(config.(field_name))
        value = config.(field_name);
    end
end
