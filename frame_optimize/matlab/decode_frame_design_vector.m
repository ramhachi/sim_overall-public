function design = decode_frame_design_vector(x, schema)
% DECODE_FRAME_DESIGN_VECTOR Convert raw optimizer variables into named data.
%
% V1 is kept for compatibility with the legacy 18-variable vector. V2 uses
% frame_design_schema_v2() and never requires positional x(6:18) access in
% callers.

    if nargin < 2 || isempty(schema)
        schema = struct('schema_version', 'frame_design/v1');
    end
    if ~isfield(schema, 'schema_version') || isempty(schema.schema_version)
        schema.schema_version = 'frame_design/v1';
    end

    x = reshape(x, 1, []);

    switch char(schema.schema_version)
        case 'frame_design/v2'
            design = decodeV2(x, schema);
        otherwise
            design = decodeV1(x);
    end
end

function design = decodeV1(x)
    requireLength(x, 18, 'frame_design/v1');

    design = struct();
    design.schema_version = 'frame_design/v1';
    design.pipe_group_indices_vector = round(x(1:5));
    design.shape_params = x(6:9);
    design.distance_thresholds = x(10:12);
    design.node_toggles = round(x(13:14));
    design.node_ratios = x(15:18);
end

function design = decodeV2(x, schema)
    requireLength(x, schema.num_variables, schema.schema_version);

    values = struct();
    for i = 1:numel(schema.variables)
        var = schema.variables(i);
        value = x(i);
        value = min(max(value, var.lower), var.upper);
        if var.is_integer
            value = round(value);
        end
        values.(var.name) = value;
    end

    pipe_group_indices = zeros(1, schema.pipe_group_count);
    pipe_group_indices_by_name = struct();
    for i = 1:schema.pipe_group_count
        group_name = schema.group_names{i};
        field_name = ['pipe_' group_name];
        pipe_group_indices(i) = values.(field_name);
        pipe_group_indices_by_name.(group_name) = values.(field_name);
    end

    geom = values;
    for i = 1:schema.pipe_group_count
        field_name = ['pipe_' schema.group_names{i}];
        if isfield(geom, field_name)
            geom = rmfield(geom, field_name);
        end
    end

    design = struct();
    design.schema_version = 'frame_design/v2';
    design.values = values;
    design.geometry = geom;
    design.group_names = schema.group_names;
    design.pipe_group_indices_vector = pipe_group_indices;
    design.pipe_group_indices = pipe_group_indices_by_name;
    design.distance_thresholds = [ ...
        values.front_distance_threshold_m, ...
        values.side_distance_threshold_m, ...
        values.rear_distance_threshold_m];
    design.node_toggles = [0, values.rear_mid_on];
    design.node_ratios = [0.5, 0.5, values.rear_mid_x_ratio, values.rear_mid_z_ratio];
end

function requireLength(x, required, version)
    if numel(x) < required
        error('decode_frame_design_vector:shortVector', ...
            '%s requires %d variables, got %d.', version, required, numel(x));
    end
end
