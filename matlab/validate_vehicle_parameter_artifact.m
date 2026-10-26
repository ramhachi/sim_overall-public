function report = validate_vehicle_parameter_artifact(artifact_or_path)
% VALIDATE_VEHICLE_PARAMETER_ARTIFACT Validate optimizer artifact contracts.
%
% report = validate_vehicle_parameter_artifact(artifact_or_path)
%
% Accepts either a decoded JSON struct or a JSON path. The report contains
% issue codes and messages instead of throwing so callers can present all
% detected contract violations together.

artifact = load_artifact(artifact_or_path);
schema = build_full_vehicle_design_schema();
issues = empty_issues();

if ~isfield(artifact, 'params') || ~isstruct(artifact.params)
    issues = add_issue(issues, 'missing_params', ...
        'Artifact must contain a params struct.');
    report = make_report(schema, issues);
    return;
end

issues = validate_design_vector(artifact, schema, issues);
issues = validate_alignment(artifact.params, issues);
issues = validate_pou_symmetry(artifact.params, issues);
report = make_report(schema, issues);
end

function artifact = load_artifact(artifact_or_path)
if ischar(artifact_or_path) || isstring(artifact_or_path)
    artifact = jsondecode(fileread(artifact_or_path));
elseif isstruct(artifact_or_path)
    artifact = artifact_or_path;
else
    error('validate_vehicle_parameter_artifact:invalidInput', ...
        'Input must be an artifact struct or JSON file path.');
end
end

function issues = validate_design_vector(artifact, schema, issues)
if ~isfield(artifact, 'design_vector') || isempty(artifact.design_vector)
    issues = add_issue(issues, 'missing_design_vector', ...
        'Artifact must contain a canonical design_vector.');
    return;
end

design_vector = artifact.design_vector(:);
expected_count = numel(schema.variable_names);
% Hardpoint offsets (HP_*) were appended to the schema and default to the
% reference geometry, so an artifact that predates only them is still valid.
n = numel(design_vector);
if n < expected_count && isfield(artifact, 'variable_names') ...
        && all(startsWith(schema.variable_names(n + 1:end), 'HP_')) ...
        && isequal(cellstr(string(artifact.variable_names(:)')), schema.variable_names(1:n))
    schema.variable_names = schema.variable_names(1:n);
    schema.variable_bounds = schema.variable_bounds(1:n, :);
    expected_count = n;
end
if numel(design_vector) ~= expected_count
    issues = add_issue(issues, 'design_vector_length_mismatch', sprintf( ...
        'Expected %d variables for %s, received %d.', ...
        expected_count, schema.version, numel(design_vector)));
    return;
end

if ~isfield(artifact, 'variable_names') || numel(artifact.variable_names) ~= expected_count
    issues = add_issue(issues, 'variable_names_length_mismatch', sprintf( ...
        'Expected %d variable_names entries.', expected_count));
else
    actual_names = cellstr(string(artifact.variable_names(:)'));
    if ~isequal(actual_names, schema.variable_names)
        issues = add_issue(issues, 'variable_names_mismatch', ...
            'variable_names does not match the canonical optimizer schema.');
    end
end

lower = schema.variable_bounds(:, 1);
upper = schema.variable_bounds(:, 2);
out_of_bounds = find(design_vector < lower | design_vector > upper);
for i = reshape(out_of_bounds, 1, [])
    issues = add_issue(issues, 'design_value_out_of_bounds', sprintf( ...
        '%s=%.12g is outside [%.12g, %.12g].', ...
        schema.variable_names{i}, design_vector(i), lower(i), upper(i)));
end
end

function issues = validate_alignment(params, issues)
if ~isfield(params, 'suspension')
    issues = add_issue(issues, 'missing_suspension', ...
        'params.suspension is required.');
    return;
end

suspension = params.suspension;
if isfield(suspension, 'static_toe')
    toe = suspension.static_toe(:)';
    if numel(toe) ~= 4 || ~is_odd_pair(toe(1:2)) || ~is_odd_pair(toe(3:4))
        issues = add_issue(issues, 'static_toe_not_mirrored', ...
            'static_toe must use mirrored left/right signs on each axle.');
    end
end

if isfield(suspension, 'static_camber')
    camber = suspension.static_camber(:)';
    if numel(camber) ~= 4 || ~is_even_pair(camber(1:2)) || ~is_even_pair(camber(3:4))
        issues = add_issue(issues, 'static_camber_not_symmetric', ...
            'static_camber must be left/right symmetric on each axle.');
    end
end
end

function issues = validate_pou_symmetry(params, issues)
if ~isfield(params, 'suspension')
    return;
end

suspension = params.suspension;
issues = validate_pou_field(suspension, issues, 'pou_jack_c1', true);
issues = validate_pou_field(suspension, issues, 'pou_jack_c2', false);
issues = validate_pou_field(suspension, issues, 'pou_camber_c1', true);
issues = validate_pou_field(suspension, issues, 'pou_camber_c2', false);
end

function issues = validate_pou_field(suspension, issues, field_name, is_odd)
if ~isfield(suspension, field_name)
    return;
end

value = suspension.(field_name)(:)';
valid = numel(value) == 4 && all(abs(value(3:4)) <= 1e-12);
if valid && is_odd
    valid = is_odd_pair(value(1:2));
elseif valid
    valid = is_even_pair(value(1:2));
end

if ~valid
    issues = add_issue(issues, 'pou_not_front_symmetric', sprintf( ...
        '%s does not match the canonical front-symmetric POU contract.', field_name));
end
end

function tf = is_odd_pair(value)
tf = numel(value) == 2 && abs(value(1) + value(2)) <= 1e-12;
end

function tf = is_even_pair(value)
tf = numel(value) == 2 && abs(value(1) - value(2)) <= 1e-12;
end

function issues = empty_issues()
issues = struct('code', {}, 'message', {});
end

function issues = add_issue(issues, code, message)
issues(end + 1) = struct('code', code, 'message', message);
end

function report = make_report(schema, issues)
report = struct( ...
    'schema_version', schema.version, ...
    'is_valid', isempty(issues), ...
    'issues', issues);
end
