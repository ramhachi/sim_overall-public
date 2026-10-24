function [seed_matrix, report] = collect_vehicle_ga_seeds(seed_paths, schema)
% COLLECT_VEHICLE_GA_SEEDS Import canonical and named legacy GA solutions.

center = mean(schema.variable_bounds, 2)';
seed_matrix = zeros(0, numel(schema.variable_names));
report = struct('path', {}, 'status', {}, 'reason', {}, ...
    'matched_variables', {}, 'clamped_variables', {});

for i = 1:numel(seed_paths)
    path = char(seed_paths{i});
    entry = struct('path', path, 'status', 'rejected', 'reason', '', ...
        'matched_variables', 0, 'clamped_variables', {{}});
    if ~isfile(path)
        entry.reason = 'file_not_found';
        report(end + 1) = entry; %#ok<AGROW>
        continue;
    end

    try
        artifact = jsondecode(fileread(path));
        [vector, names] = extract_named_vector(artifact);
    catch import_error
        entry.reason = ['decode_error:', import_error.identifier];
        report(end + 1) = entry; %#ok<AGROW>
        continue;
    end

    if isempty(vector) || isempty(names) || numel(vector) ~= numel(names)
        entry.reason = 'missing_named_design_vector';
        report(end + 1) = entry; %#ok<AGROW>
        continue;
    end

    candidate = center;
    matched = false(1, numel(schema.variable_names));
    for j = 1:numel(names)
        canonical_index = find(strcmp(schema.variable_names, char(names{j})), 1);
        if ~isempty(canonical_index)
            candidate(canonical_index) = vector(j);
            matched(canonical_index) = true;
        end
    end
    entry.matched_variables = nnz(matched);
    if entry.matched_variables < 10
        entry.reason = 'insufficient_name_overlap';
        report(end + 1) = entry; %#ok<AGROW>
        continue;
    end

    lower = schema.variable_bounds(:, 1)';
    upper = schema.variable_bounds(:, 2)';
    clamped = find(candidate < lower | candidate > upper);
    candidate = min(max(candidate, lower), upper);
    entry.clamped_variables = schema.variable_names(clamped);
    if isequal(cellstr(string(names(:)')), schema.variable_names) && isempty(clamped)
        entry.status = 'accepted_exact';
        entry.reason = 'canonical_schema_match';
    else
        entry.status = 'accepted_migrated';
        entry.reason = 'name_mapped_missing_values_centered_and_bounds_clamped';
    end

    if isempty(seed_matrix) || ~ismember(candidate, seed_matrix, 'rows')
        seed_matrix(end + 1, :) = candidate; %#ok<AGROW>
    else
        entry.status = 'rejected';
        entry.reason = 'duplicate_after_migration';
    end
    report(end + 1) = entry; %#ok<AGROW>
end
end

function [vector, names] = extract_named_vector(artifact)
vector = [];
names = {};
if isfield(artifact, 'design_vector') && isfield(artifact, 'variable_names')
    vector = double(artifact.design_vector(:)');
    names = cellstr(string(artifact.variable_names(:)'));
    return;
end

if ~isfield(artifact, 'optimization') || ~isfield(artifact.optimization, 'stages')
    return;
end
stages = artifact.optimization.stages;
if iscell(stages)
    stage = stages{end};
elseif isstruct(stages)
    stage = stages(end);
else
    return;
end
if isfield(stage, 'design_vector') && isfield(stage, 'variable_names')
    vector = double(stage.design_vector(:)');
    names = cellstr(string(stage.variable_names(:)'));
end
end
