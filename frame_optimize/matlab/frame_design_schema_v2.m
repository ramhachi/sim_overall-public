function schema = frame_design_schema_v2(catalog_count)
% FRAME_DESIGN_SCHEMA_V2 Central definition for the V2 frame design vector.
%
% The first variables are pipe catalog indices. The remaining variables are
% geometric or topology controls, all in SI units unless noted otherwise.

    if nargin < 1 || isempty(catalog_count)
        catalog_count = 1;
    end
    catalog_count = max(1, catalog_count);

    vars = struct('name', {}, 'unit', {}, 'lower', {}, 'upper', {}, ...
        'initial', {}, 'is_integer', {}, 'description', {});

    group_names = { ...
        'front_hoop', ...
        'main_hoop', ...
        'side_impact', ...
        'front_bulkhead', ...
        'fbh_support', ...
        'front_susp_mount', ...
        'mh_bracing', ...
        'mh_brace_support', ...
        'shoulder_bar', ...
        'component_protection', ...
        'diff_mount', ...
        'rear_box', ...
        'secondary_brace'};

    initial_pipe_by_group = [6, 6, 5, 5, 5, 19, 5, 19, 6, 19, 19, 19, 19];
    for i = 1:numel(group_names)
        vars(end + 1) = makeVar( ... %#ok<AGROW>
            ['pipe_' group_names{i}], 'catalog_index', 1, catalog_count, ...
            min(initial_pipe_by_group(i), catalog_count), true, ['Catalog index for ' group_names{i}]);
    end

    vars(end + 1) = makeVar('fbh_half_width_m', 'm', 0.17, 0.28, 0.20, false, 'Front bulkhead half width');
    vars(end + 1) = makeVar('fbh_height_m', 'm', 0.30, 0.55, 0.40, false, 'Front bulkhead height');
    vars(end + 1) = makeVar('fbh_x_shift_m', 'm', -0.05, 0.12, 0.03, false, 'FBH longitudinal shift from suspension baseline');
    vars(end + 1) = makeVar('fbh_z_offset_m', 'm', -0.12, 0.06, -0.05, false, 'FBH vertical offset from suspension baseline');

    vars(end + 1) = makeVar('fh_half_width_m', 'm', 0.28, 0.48, 0.32, false, 'Front hoop half width');
    vars(end + 1) = makeVar('fh_top_z_m', 'm', 0.70, 1.05, 0.95, false, 'Front hoop top height coordinate');
    vars(end + 1) = makeVar('fh_lower_z_m', 'm', 0.02, 0.12, 0.05, false, 'Front hoop lower node height coordinate');

    vars(end + 1) = makeVar('si_half_width_m', 'm', 0.28, 0.45, 0.35, false, 'Side impact half width');
    vars(end + 1) = makeVar('si_upper_delta_m', 'm', 0.290, 0.345, 0.320, false, 'Upper side impact height above lower side impact');
    vars(end + 1) = makeVar('si_x_ratio', 'ratio', 0.25, 0.75, 0.35, false, 'Side impact X location between FH and MH');

    vars(end + 1) = makeVar('mh_x_offset_from_fh_m', 'm', 0.55, 0.85, 0.60, false, 'Main hoop X offset from front hoop');
    vars(end + 1) = makeVar('mh_base_half_width_m', 'm', 0.20, 0.36, 0.30, false, 'Main hoop lower half width');
    vars(end + 1) = makeVar('mh_mid_half_width_m', 'm', 0.30, 0.50, 0.40, false, 'Main hoop middle half width');
    vars(end + 1) = makeVar('mh_shoulder_half_width_m', 'm', 0.10, 0.25, 0.20, false, 'Main hoop shoulder/brace half width');
    vars(end + 1) = makeVar('mh_top_z_m', 'm', 1.25, 1.45, 1.40, false, 'Main hoop top height coordinate');
    vars(end + 1) = makeVar('shoulder_bar_z_m', 'm', 1.02, 1.30, 1.15, false, 'Shoulder harness bar height coordinate');
    vars(end + 1) = makeVar('mh_brace_attach_z_m', 'm', 1.30, 1.50, 1.35, false, 'Main hoop brace attachment height coordinate');

    vars(end + 1) = makeVar('mh_brace_end_x_offset_m', 'm', 0.45, 0.75, 0.50, false, 'Brace trapezoid end offset behind MH');
    vars(end + 1) = makeVar('mh_brace_end_top_half_width_m', 'm', 0.20, 0.42, 0.28, false, 'Brace end trapezoid upper half width');
    vars(end + 1) = makeVar('mh_brace_end_bottom_half_width_m', 'm', 0.22, 0.48, 0.32, false, 'Brace end trapezoid lower half width');
    vars(end + 1) = makeVar('mh_brace_end_upper_z_m', 'm', 0.90, 1.15, 0.90, false, 'Brace end trapezoid upper height coordinate');
    vars(end + 1) = makeVar('mh_brace_end_lower_z_m', 'm', 0.04, 0.25, 0.08, false, 'Brace end trapezoid lower height coordinate');

    vars(end + 1) = makeVar('rear_free_x_offset_m', 'm', 0.10, 0.45, 0.25, false, 'Rear free region extension behind brace trapezoid');
    vars(end + 1) = makeVar('rear_free_half_width_m', 'm', 0.22, 0.45, 0.30, false, 'Rear free region half width');
    vars(end + 1) = makeVar('rear_free_upper_z_m', 'm', 0.40, 0.85, 0.55, false, 'Rear free region upper height coordinate');

    vars(end + 1) = makeVar('front_distance_threshold_m', 'm', 0.40, 0.80, 0.60, false, 'Optional front connection radius');
    vars(end + 1) = makeVar('side_distance_threshold_m', 'm', 0.40, 0.80, 0.65, false, 'Optional side connection radius');
    vars(end + 1) = makeVar('rear_distance_threshold_m', 'm', 0.40, 0.80, 0.60, false, 'Optional rear connection radius');

    vars(end + 1) = makeVar('rear_mid_on', 'bool', 0, 1, 0, true, 'Enable a rear free-region mid plane');
    vars(end + 1) = makeVar('rear_mid_x_ratio', 'ratio', 0.20, 0.80, 0.50, false, 'Rear mid plane X interpolation');
    vars(end + 1) = makeVar('rear_mid_z_ratio', 'ratio', 0.20, 0.80, 0.50, false, 'Rear mid plane Z interpolation');

    schema = struct();
    schema.schema_version = 'frame_design/v2';
    schema.variables = vars;
    schema.group_names = group_names;
    schema.pipe_group_count = numel(group_names);
    schema.num_variables = numel(vars);
    schema.lower_bounds = [vars.lower];
    schema.upper_bounds = [vars.upper];
    schema.initial = [vars.initial];
    schema.integer_indices = find([vars.is_integer]);
end

function v = makeVar(name, unit, lower, upper, initial, is_integer, description)
    v = struct();
    v.name = name;
    v.unit = unit;
    v.lower = lower;
    v.upper = upper;
    v.initial = min(max(initial, lower), upper);
    v.is_integer = logical(is_integer);
    v.description = description;
end
