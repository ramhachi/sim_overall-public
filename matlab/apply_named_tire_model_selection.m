function [base, selected_model, pinn_catalog] = apply_named_tire_model_selection(base, tire_model_family, tire_model_name, project_root)
% APPLY_NAMED_TIRE_MODEL_SELECTION Apply a fixed tire runtime configuration.
%
% [base, selected_model, pinn_catalog] = apply_named_tire_model_selection( ...
%     base, tire_model_family, tire_model_name, project_root)
%
% Inputs:
%   base               - Vehicle parameter struct
%   tire_model_family  - must be 'pinn' for production runtime
%   tire_model_name    - Deployable PINN tire model name when family='pinn'
%   project_root       - Repository root
%
% Outputs:
%   base               - Updated vehicle parameters
%   selected_model     - Selected PINN model metadata (empty otherwise)
%   pinn_catalog       - Deployable PINN tire model catalog for this repo

if nargin < 1 || isempty(base)
    base = struct();
end
if nargin < 2 || isempty(tire_model_family)
    tire_model_family = 'pinn';
end
if nargin < 3
    tire_model_name = '';
end
if nargin < 4 || isempty(project_root)
    project_root = pwd;
    if ~exist(fullfile(project_root, 'docker-compose.yml'), 'file')
        project_root = fileparts(project_root);
    end
end

if ~isfield(base, 'tire') || isempty(base.tire)
    base.tire = struct();
end

family = lower(strtrim(char(tire_model_family)));
name = strtrim(char(tire_model_name));
pinn_catalog = scan_deployable_pinn_catalog(project_root);

switch family
    case 'tmeasy'
        error('TMeasy tire runtime is disabled for production. Select a deployable PINN model.');

    case 'magic_formula'
        error('Magic Formula tire runtime is disabled for production. Select a deployable PINN model.');

    case 'pinn'
        if isempty(name)
            error('PINN tire model selected but tire_model_name is empty.');
        end

        selected_model = find_catalog_entry(pinn_catalog, name);
        if isempty(selected_model)
            selected_model = build_catalog_entry(project_root, name);
        end

        base.tire.model_type = 'pinn';
        base.tire.model_name = selected_model.name;
        base.tire.model_dir = selected_model.model_dir;
        base.tire.model_manifest = selected_model.model_manifest;
        base.tire.mu_x_max = selected_model.mu_x_max;
        base.tire.mu_y_max = selected_model.mu_y_max;

    otherwise
        error('tire_model_family must be ''pinn'' for production runtime.');
end

if ~isfield(base.tire, 'mu_x_max') || isempty(base.tire.mu_x_max)
    base.tire.mu_x_max = 1.6;
end
if ~isfield(base.tire, 'mu_y_max') || isempty(base.tire.mu_y_max)
    base.tire.mu_y_max = 2.0;
end
end

function models = scan_deployable_pinn_catalog(project_root)
models = struct( ...
    'name', {}, ...
    'display_name', {}, ...
    'model_dir', {}, ...
    'model_manifest', {}, ...
    'manifest_host', {}, ...
    'mu_x_max', {}, ...
    'mu_y_max', {}, ...
    'r2_min', {});

model_root = fullfile(project_root, 'output', 'tire_models');
if ~exist(model_root, 'dir')
    return;
end

valid_datasets = scan_trainable_tire_datasets(project_root);

entries = dir(model_root);
for i = 1:numel(entries)
    if ~entries(i).isdir || startsWith(entries(i).name, '.')
        continue;
    end

    manifest_path = fullfile(entries(i).folder, entries(i).name, 'manifest.json');
    if ~isfile(manifest_path)
        continue;
    end

    try
        manifest = jsondecode(fileread(manifest_path));
    catch
        continue;
    end

    if ~isfield(manifest, 'deployable') || ~manifest.deployable
        continue;
    end

    tire_name = '';
    if isfield(manifest, 'tire_name') && ~isempty(manifest.tire_name)
        tire_name = char(manifest.tire_name);
    elseif isfield(manifest, 'dataset') && isfield(manifest.dataset, 'identifier') ...
            && ~isempty(manifest.dataset.identifier)
        tire_name = char(manifest.dataset.identifier);
    end
    if isempty(tire_name)
        continue;
    end
    if ~isempty(valid_datasets) && ~any(strcmp(valid_datasets, tire_name))
        continue;
    end

    if ~isfield(manifest, 'dataset') || ~isfield(manifest.dataset, 'identifier')
        continue;
    end
    if ~strcmp(char(manifest.dataset.identifier), tire_name)
        continue;
    end

    models(end + 1) = build_catalog_entry(project_root, tire_name, manifest_path, manifest); %#ok<AGROW>
end

if isempty(models)
    return;
end

[~, order] = sort({models.name});
models = models(order);
end

function selected = find_catalog_entry(models, tire_name)
selected = struct([]);
if isempty(models)
    return;
end

matches = strcmp({models.name}, tire_name);
if any(matches)
    selected = models(find(matches, 1, 'first'));
end
end

function entry = build_catalog_entry(project_root, tire_name, manifest_path, manifest)
if nargin < 3 || isempty(manifest_path)
    manifest_path = fullfile(project_root, 'output', 'tire_models', tire_name, 'manifest.json');
end
if nargin < 4 || isempty(manifest)
    if ~isfile(manifest_path)
        error('PINN manifest not found: %s', manifest_path);
    end
    manifest = jsondecode(fileread(manifest_path));
end

if ~isfield(manifest, 'deployable') || ~manifest.deployable
    error('PINN tire model is not deployable: %s', tire_name);
end

[mu_x_max, mu_y_max] = estimate_planner_mu(fullfile(fileparts(manifest_path), 'test_vectors.json'));

display_name = tire_name;
if isfield(manifest, 'dataset') && isfield(manifest.dataset, 'display_name') ...
        && ~isempty(manifest.dataset.display_name)
    display_name = char(manifest.dataset.display_name);
end

r2_min = NaN;
if isfield(manifest, 'validation') && isfield(manifest.validation, 'r2_min')
    r2_min = manifest.validation.r2_min;
end

entry = struct( ...
    'name', tire_name, ...
    'display_name', display_name, ...
    'model_dir', sprintf('/workspace/output/tire_models/%s', tire_name), ...
    'model_manifest', sprintf('/workspace/output/tire_models/%s/manifest.json', tire_name), ...
    'manifest_host', manifest_path, ...
    'mu_x_max', mu_x_max, ...
    'mu_y_max', mu_y_max, ...
    'r2_min', r2_min);

assert_pinn_runtime_assets(manifest_path, manifest, tire_name);
end

function assert_pinn_runtime_assets(manifest_path, manifest, tire_name)
runtime_model = 'pinn_runtime.json';
if isfield(manifest, 'runtime_files') && isfield(manifest.runtime_files, 'runtime_model') ...
        && ~isempty(manifest.runtime_files.runtime_model)
    runtime_model = char(manifest.runtime_files.runtime_model);
end

runtime_host = fullfile(fileparts(manifest_path), runtime_model);
if ~isfile(runtime_host)
    error('PINN runtime file not found for %s: %s', tire_name, runtime_host);
end
end

function [mu_x_max, mu_y_max] = estimate_planner_mu(test_vectors_path)
mu_x_max = 1.6;
mu_y_max = 2.0;

if ~isfile(test_vectors_path)
    return;
end

try
    vectors = jsondecode(fileread(test_vectors_path));
    inputs = json_array_to_matrix(vectors.inputs);
    outputs = json_array_to_matrix(vectors.outputs);
catch
    return;
end

if size(inputs, 2) < 3 || size(outputs, 2) < 2
    return;
end

fz = abs(inputs(:, 3));
valid = isfinite(fz) & (fz > 50.0);
if ~any(valid)
    return;
end

mu_x_samples = abs(outputs(valid, 1)) ./ fz(valid);
mu_y_samples = abs(outputs(valid, 2)) ./ fz(valid);

mu_x_max = robust_positive_quantile(mu_x_samples, mu_x_max);
mu_y_max = robust_positive_quantile(mu_y_samples, mu_y_max);
end

function matrix = json_array_to_matrix(value)
if isnumeric(value)
    matrix = value;
    return;
end

if ~iscell(value) || isempty(value)
    matrix = [];
    return;
end

row_count = numel(value);
col_count = numel(value{1});
matrix = zeros(row_count, col_count);
for i = 1:row_count
    row = value{i};
    if iscell(row)
        row = cell2mat(row);
    end
    matrix(i, :) = double(row(:)).';
end
end

function q = robust_positive_quantile(samples, fallback_value)
samples = samples(isfinite(samples) & (samples > 0.0));
if isempty(samples)
    q = fallback_value;
    return;
end

samples = sort(samples);
index = max(1, min(numel(samples), ceil(0.95 * numel(samples))));
q = samples(index);
q = min(max(q, 0.5), 4.0);
end

function tire = remove_optional_tire_fields(tire, field_names)
for i = 1:numel(field_names)
    if isfield(tire, field_names{i})
        tire = rmfield(tire, field_names{i});
    end
end
end

function datasets = scan_trainable_tire_datasets(project_root)
data_dir = fullfile(project_root, 'data', 'tiredata');
datasets = {};
if ~exist(data_dir, 'dir')
    return;
end

manifest_entries = load_raw_tire_manifest_entries(project_root);
files = dir(fullfile(data_dir, '**', '*.dat'));
for i = 1:numel(files)
    file_path = fullfile(files(i).folder, files(i).name);
    fid = fopen(file_path, 'r');
    if fid < 0
        continue;
    end

    header_line = fgetl(fid);
    fclose(fid);
    if ~ischar(header_line)
        continue;
    end

    manifest_tire_name = lookup_manifest_tire_name(files(i).name, manifest_entries);
    if ~isempty(manifest_tire_name)
        datasets{end + 1} = normalize_tire_name(manifest_tire_name); %#ok<AGROW>
        continue;
    end

    tire_tokens = regexp(header_line, 'Tire_Name=([^;]+)', 'tokens', 'once');
    if ~isempty(tire_tokens)
        datasets{end + 1} = normalize_tire_name(strtrim(tire_tokens{1})); %#ok<AGROW>
        continue;
    end

    project_tokens = regexp(header_line, 'Project\s+(\d+)', 'tokens', 'once');
    if ~isempty(project_tokens)
        datasets{end + 1} = sprintf('Project%s_UnknownTire', project_tokens{1}); %#ok<AGROW>
    end
end

datasets = unique(datasets);
end

function manifest_entries = load_raw_tire_manifest_entries(project_root)
manifest_entries = struct('name', {}, 'tire_name', {});
manifest_path = fullfile(project_root, 'data', 'tiredata', 'raw_ttc_manifest.json');
if ~isfile(manifest_path)
    return;
end

try
    manifest = jsondecode(fileread(manifest_path));
catch
    return;
end

if ~isfield(manifest, 'files') || isempty(manifest.files)
    return;
end

files_field = manifest.files;
if isstruct(files_field) && isfield(files_field, 'name')
    manifest_entries = files_field(:);
    return;
end

if ~isstruct(files_field)
    return;
end

file_names = fieldnames(files_field);
manifest_entries = repmat(struct('name', '', 'tire_name', ''), numel(file_names), 1);
for i = 1:numel(file_names)
    file_name = file_names{i};
    entry = files_field.(file_name);
    manifest_entries(i).name = file_name;
    if isstruct(entry) && isfield(entry, 'tire_name') && ~isempty(entry.tire_name)
        manifest_entries(i).tire_name = entry.tire_name;
    end
end
end

function tire_name = lookup_manifest_tire_name(file_name, manifest_entries)
tire_name = '';
if isempty(manifest_entries)
    return;
end

for i = 1:numel(manifest_entries)
    if isfield(manifest_entries(i), 'name') && strcmp(manifest_entries(i).name, file_name)
        if isfield(manifest_entries(i), 'tire_name') && ~isempty(manifest_entries(i).tire_name)
            tire_name = strtrim(char(manifest_entries(i).tire_name));
        end
        return;
    end
end
end

function normalized = normalize_tire_name(raw_name)
normalized = strtrim(raw_name);
normalized = strrep(normalized, ' ', '_');
normalized = strrep(normalized, ',', '');
normalized = strrep(normalized, '.', 'p');
normalized = strrep(normalized, '×', 'X');
end
