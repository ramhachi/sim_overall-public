function result = save_frame_optimization_artifact(varargin)
% SAVE_FRAME_OPTIMIZATION_ARTIFACT Persist frame optimization results into a unified vehicle artifact JSON.
%
% Additive / hardened behavior:
%   - Existing OutputPath artifact is loaded and preserved even when BaseArtifactPath is empty.
%   - params.frame is updated additively: the prior frame data is pushed into params.frame.history
%     so provenance is never silently discarded.
%   - BestDesignVector is validated: must be a vector with at least 5 elements when non-empty;
%     a warning is raised when shorter than the full 18-element expected length.
%   - K_chassis is only mirrored from torsional_stiffness when the value is finite and > 0.

p = inputParser;
addParameter(p, 'BaseArtifactPath', '', @(x) ischar(x) || isstring(x));
addParameter(p, 'OutputPath', '', @(x) ischar(x) || isstring(x));
addParameter(p, 'CurrentParams', struct(), @isstruct);
addParameter(p, 'BestDesignVector', [], @isnumeric);
addParameter(p, 'BestObjectiveValue', NaN, @isnumeric);
addParameter(p, 'Catalog', struct([]), @isstruct);
addParameter(p, 'ElementsGeom', struct(), @isstruct);
addParameter(p, 'Nodes', [], @isnumeric);
addParameter(p, 'FinalResults', struct(), @isstruct);
addParameter(p, 'Config', struct(), @isstruct);
addParameter(p, 'OptimizationOutput', struct(), @isstruct);
addParameter(p, 'ScriptName', 'frame_optimize/examples/run_frame_optimization.m', @(x) ischar(x) || isstring(x));
parse(p, varargin{:});
opts = p.Results;

baseArtifactPath = char(opts.BaseArtifactPath);
outputPath = char(opts.OutputPath);
if isempty(outputPath)
    error('save_frame_optimization_artifact:missingOutputPath', 'OutputPath is required.');
end

% Validate BestDesignVector before any further processing.
validateBestDesignVector(opts.BestDesignVector);

% Load base artifact: prefer explicit BaseArtifactPath, then fall back to the
% existing OutputPath so that prior data is never silently discarded.
artifact = struct();
params = create_vehicle_baseline_template();
loadedPath = '';
if ~isempty(baseArtifactPath) && isfile(baseArtifactPath)
    artifact = load_vehicle_artifact(baseArtifactPath);
    params = merge_nested_structs(params, artifact.params);
    loadedPath = baseArtifactPath;
elseif isfile(outputPath)
    artifact = load_vehicle_artifact(outputPath);
    params = merge_nested_structs(params, artifact.params);
    loadedPath = outputPath;
end
params = merge_nested_structs(params, opts.CurrentParams);

frameSection = buildFrameSection(opts);

% Additive frame update: push the prior frame data into history before replacing it.
if isfield(params, 'frame') && isstruct(params.frame) && ~isempty(fieldnames(params.frame))
    frameSection = mergeFrameHistory(frameSection, params.frame);
end

% Mirror K_chassis only when the stiffness value is finite and strictly positive.
if isfield(frameSection, 'performance') ...
        && isfield(frameSection.performance, 'torsional_stiffness_Nm_per_deg')
    K_val = frameSection.performance.torsional_stiffness_Nm_per_deg;
    if isscalar(K_val) && isfinite(K_val) && K_val > 0
        params.suspension.K_chassis = K_val;
    end
end
params.frame = frameSection;

stageEntry = struct();
stageEntry.name = 'frame_optimization';
stageEntry.optimizer = getStructFieldOrDefault(opts.Config, 'optimizer_mode', 'mixed_ga');
stageEntry.params = params;
stageEntry.result = buildStageResult(opts, frameSection);

artifact = build_vehicle_artifact(params, ...
    'base_artifact', artifact, ...
    'stage_entry', stageEntry);
outputDir = fileparts(outputPath);
if ~isempty(outputDir) && ~exist(outputDir, 'dir')
    mkdir(outputDir);
end
save_vehicle_artifact(outputPath, artifact);

result = struct();
result.output_path = outputPath;
result.updated_existing_artifact = ~isempty(loadedPath);
result.loaded_base_path = loadedPath;
result.params = params;
result.frame = frameSection;
end

% ---------------------------------------------------------------------------
% Input validation
% ---------------------------------------------------------------------------

function validateBestDesignVector(vec)
% Validate BestDesignVector: must be a numeric vector of at least 5 elements when non-empty.
if isempty(vec)
    return;
end
if ~isvector(vec)
    error('save_frame_optimization_artifact:invalidDesignVector', ...
        'BestDesignVector must be a numeric vector; received array of size [%s].', ...
        num2str(size(vec)));
end
MIN_LEN = 5;   % minimum: at least one pipe group index per group
FULL_LEN = 18; % expected full length for all design variable slices
if numel(vec) < MIN_LEN
    error('save_frame_optimization_artifact:shortDesignVector', ...
        'BestDesignVector has %d element(s); at least %d are required for pipe group indices.', ...
        numel(vec), MIN_LEN);
end
if numel(vec) < FULL_LEN
    warning('save_frame_optimization_artifact:shortDesignVector', ...
        'BestDesignVector has %d elements; full design expects %d. Tail design-variable slices will be empty.', ...
        numel(vec), FULL_LEN);
end
end

% ---------------------------------------------------------------------------
% Additive frame history merge
% ---------------------------------------------------------------------------

function newFrame = mergeFrameHistory(newFrame, oldFrame)
% Push the prior frame state into newFrame.history (chronological, oldest first).
existingHistory = {};
if isfield(oldFrame, 'history') && iscell(oldFrame.history) && ~isempty(oldFrame.history)
    existingHistory = oldFrame.history;
end
% Snapshot the old frame without its own history entry to avoid deep nesting.
oldSnapshot = removeFieldSafe(oldFrame, 'history');
newFrame.history = [existingHistory, {oldSnapshot}];
end

function s = removeFieldSafe(s, fname)
if isfield(s, fname)
    s = rmfield(s, fname);
end
end

function frameSection = buildFrameSection(opts)
bestX = reshape(opts.BestDesignVector, 1, []);
selectedPipes = buildSelectedPipes(bestX, opts.Catalog, opts.Config);

performance = struct();
performance.mass_kg = getStructFieldOrDefault(opts.FinalResults, 'mass', NaN);
performance.cg_z_m = getStructFieldOrDefault(opts.FinalResults, 'cg_z', NaN);
performance.torsional_stiffness_Nm_per_deg = getStructFieldOrDefault(opts.FinalResults, 'K_torsion', NaN);
performance.objective_value = opts.BestObjectiveValue;
performance.total_length_m = getStructFieldOrDefault(opts.FinalResults, 'L_total', NaN);
performance.num_elements = getStructFieldOrDefault(opts.FinalResults, 'num_elements', NaN);
performance.penalties = struct( ...
    'stiffness', getStructFieldOrDefault(opts.FinalResults, 'P_stiff', NaN), ...
    'mass_target', getStructFieldOrDefault(opts.FinalResults, 'P_mass_target', NaN), ...
    'material', getStructFieldOrDefault(opts.FinalResults, 'material_penalty', NaN), ...
    'geometry', getStructFieldOrDefault(opts.FinalResults, 'geometry_penalty', NaN), ...
    'geometry_precheck', getStructFieldOrDefault(opts.FinalResults, 'geometry_precheck_penalty', NaN), ...
    'cg', getStructFieldOrDefault(opts.FinalResults, 'cg_penalty', NaN), ...
    'exclusion', getStructFieldOrDefault(opts.FinalResults, 'exclusion_penalty', NaN), ...
    'sparsity', getStructFieldOrDefault(opts.FinalResults, 'sparsity_penalty', NaN), ...
    'compliance', getStructFieldOrDefault(opts.FinalResults, 'compliance_penalty', NaN), ...
    'load_case_displacement', getStructFieldOrDefault(opts.FinalResults, 'case_displacement_penalty', NaN));

designVariables = buildDesignVariables(bestX, opts.Config);

geometry = struct();
geometry.nodes_m = opts.Nodes;
geometry.connectivity = getStructFieldOrDefault(opts.ElementsGeom, 'connectivity', []);
geometry.group_indices = getStructFieldOrDefault(opts.ElementsGeom, 'group_indices', []);
geometry.group_names = cellstrOrEmpty(getStructFieldOrDefault(opts.ElementsGeom, 'group_names', {}));
geometry.base_elements = getStructFieldOrDefault(opts.ElementsGeom, 'base_elements', []);

frameSection = struct();
frameSection.schema_version = frameResultSchemaVersion(opts.Config);
frameSection.updated_at = datestr(now, 'yyyy-mm-dd HH:MM:SS');
frameSection.source_script = char(opts.ScriptName);
frameSection.selected_pipes = selectedPipes;
frameSection.design_variables = designVariables;
frameSection.performance = performance;
frameSection.targets = buildTargets(opts.Config);
frameSection.objective_config = buildObjectiveConfig(opts.Config);
frameSection.geometry = geometry;
frameSection.optimization_run = buildOptimizationRun(opts.OptimizationOutput);
frameSection.load_cases = sanitizeLoadCaseResults(getStructFieldOrDefault(opts.FinalResults, 'load_case_results', struct([])));
frameSection.regulation_report = buildRegulationReport(opts.FinalResults);
if isfield(opts.FinalResults, 'provisional') && logical(opts.FinalResults.provisional)
    frameSection.provisional = true;
    if isfield(opts.FinalResults, 'provisional_reasons')
        frameSection.provisional_reasons = opts.FinalResults.provisional_reasons;
    end
end
if isfield(opts.Config, 'diff_mount_points')
    frameSection.diff_mount_points = opts.Config.diff_mount_points;
end
if isfield(opts.Config, 'diff_envelope')
    frameSection.diff_envelope = opts.Config.diff_envelope;
end
end

function selectedPipes = buildSelectedPipes(bestX, catalog, config)
selectedPipes = struct('group_index', {}, 'catalog_index', {}, 'id', {}, ...
    'group_name', {}, 'shape', {}, 'material_id', {}, 'D_outer_mm', {}, 't_mm', {});
if isempty(bestX) || isempty(catalog)
    return;
end

groupNames = {};
if isfield(config, 'design_schema') && isfield(config.design_schema, 'group_names')
    groupNames = config.design_schema.group_names;
end
if isempty(groupNames)
    groupNames = {'front_hoop', 'main_hoop', 'side_impact', 'structural', 'shoulder'};
end
if numel(bestX) < numel(groupNames)
    error('save_frame_optimization_artifact:incompleteDesignVector', ...
        'BestDesignVector has %d element(s) but %d pipe groups are defined.', ...
        numel(bestX), numel(groupNames));
end
for i = 1:numel(groupNames)
    rawIndex = bestX(i);
    catalogIndex = round(rawIndex);
    if ~isfinite(rawIndex) || abs(rawIndex - catalogIndex) > 1e-6 ...
            || catalogIndex < 1 || catalogIndex > numel(catalog)
        error('save_frame_optimization_artifact:invalidCatalogIndex', ...
            'Group "%s" has invalid catalog index %g (catalog size %d).', ...
            groupNames{i}, rawIndex, numel(catalog));
    end
    pipe = catalog(catalogIndex);
    selectedPipes(end + 1) = struct( ... %#ok<AGROW>
        'group_index', i, ...
        'catalog_index', catalogIndex, ...
        'id', getStructFieldOrDefault(pipe, 'id', sprintf('group_%d', i)), ...
        'group_name', groupNames{i}, ...
        'shape', getStructFieldOrDefault(pipe, 'shape', ''), ...
        'material_id', getStructFieldOrDefault(pipe, 'material_id', ''), ...
        'D_outer_mm', getStructFieldOrDefault(pipe, 'D_outer_mm', NaN), ...
        't_mm', getStructFieldOrDefault(pipe, 't_mm', NaN));
end
end

function targets = buildTargets(config)
targets = struct();
targets.K_target_Nm_per_deg = getStructFieldOrDefault(config, 'K_target', NaN);
targets.K_min_Nm_per_deg = getStructFieldOrDefault(config, 'K_min', NaN);
targets.mass_target_kg = getStructFieldOrDefault(config, 'mass_target', NaN);
targets.max_cg_z_m = getStructFieldOrDefault(config, 'max_cg_z', NaN);
end

function schemaVersion = frameResultSchemaVersion(config)
schemaVersion = 'frame_result/v1';
if isfield(config, 'design_schema_version') && strcmp(config.design_schema_version, 'frame_design/v2')
    schemaVersion = 'frame_result/v2';
end
end

function designVariables = buildDesignVariables(bestX, config)
designVariables = struct();
designVariables.best_vector = bestX;

if isfield(config, 'design_schema') && isfield(config.design_schema, 'schema_version') ...
        && strcmp(config.design_schema.schema_version, 'frame_design/v2')
    if exist('decode_frame_design_vector', 'file') ~= 2
        addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'frame_optimize', 'matlab'));
    end
    design = decode_frame_design_vector(bestX, config.design_schema);
    designVariables.schema_version = design.schema_version;
    designVariables.schema = config.design_schema;
    designVariables.pipe_group_indices = design.pipe_group_indices;
    designVariables.pipe_group_indices_vector = design.pipe_group_indices_vector;
    designVariables.geometry = design.geometry;
    designVariables.distance_thresholds = design.distance_thresholds;
    designVariables.node_toggles = design.node_toggles;
    designVariables.node_ratios = design.node_ratios;
    return;
end

designVariables.schema_version = 'frame_design/v1';
designVariables.pipe_group_indices = extractVectorSlice(bestX, 1, 5);
designVariables.shape_params = extractVectorSlice(bestX, 6, 9);
designVariables.distance_thresholds = extractVectorSlice(bestX, 10, 12);
designVariables.node_toggles = extractVectorSlice(bestX, 13, 14);
designVariables.node_ratios = extractVectorSlice(bestX, 15, 18);
end

function objectiveConfig = buildObjectiveConfig(config)
objectiveConfig = struct();
objectiveConfig.W_mass = getStructFieldOrDefault(config, 'W_mass', NaN);
objectiveConfig.W_cg = getStructFieldOrDefault(config, 'W_cg', NaN);
objectiveConfig.mass_ref_kg = getStructFieldOrDefault(config, 'mass_ref', NaN);
objectiveConfig.cg_ref_m = getStructFieldOrDefault(config, 'cg_ref', NaN);
objectiveConfig.gamma_below = getStructFieldOrDefault(config, 'gamma_below', NaN);
objectiveConfig.gamma_hard = getStructFieldOrDefault(config, 'gamma_hard', NaN);
objectiveConfig.compliance_penalty_scale = getStructFieldOrDefault(config, 'compliance_penalty_scale', NaN);
objectiveConfig.gamma_mass = getStructFieldOrDefault(config, 'gamma_mass', NaN);
objectiveConfig.joint_factor = getStructFieldOrDefault(config, 'joint_factor', NaN);
objectiveConfig.max_cg_z_m = getStructFieldOrDefault(config, 'max_cg_z', NaN);
objectiveConfig.sparsity_max_total_length_m = getStructFieldOrDefault(config, 'sparsity_max_total_length_m', NaN);
objectiveConfig.sparsity_max_elements = getStructFieldOrDefault(config, 'sparsity_max_elements', NaN);
objectiveConfig.sparsity_penalty_scale = getStructFieldOrDefault(config, 'sparsity_penalty_scale', NaN);
objectiveConfig.require_rear_suspension_hardpoints = getStructFieldOrDefault(config, 'require_rear_suspension_hardpoints', false);
objectiveConfig.require_total_mass_kg = getStructFieldOrDefault(config, 'require_total_mass_kg', false);
objectiveConfig.design_schema_version = getStructFieldOrDefault(config, 'design_schema_version', 'frame_design/v1');
objectiveConfig.optimizer_mode = getStructFieldOrDefault(config, 'optimizer_mode', 'mixed_ga');
objectiveConfig.random_seed = getStructFieldOrDefault(config, 'random_seed', []);
objectiveConfig.load_case_eval_mode = getStructFieldOrDefault(config, 'load_case_eval_mode', 'full');
objectiveConfig.solver_backend = getStructFieldOrDefault(config, 'solver_backend', 'tcp_java');
objectiveConfig.solver_backend_requested = getStructFieldOrDefault(config, 'solver_backend_requested', objectiveConfig.solver_backend);
objectiveConfig.fem_server_lifecycle = getStructFieldOrDefault(config, 'fem_server_lifecycle', 'managed_once');
objectiveConfig.finite_rejection_penalty = getStructFieldOrDefault(config, 'finite_rejection_penalty', false);
objectiveConfig.invalid_design_penalty = getStructFieldOrDefault(config, 'invalid_design_penalty', NaN);
objectiveConfig.case_max_displacement_m = getStructFieldOrDefault(config, 'case_max_displacement_m', NaN);
objectiveConfig.max_rear_free_pipe_length_m = getStructFieldOrDefault(config, 'max_rear_free_pipe_length_m', NaN);
objectiveConfig.enable_rear_optional_edges = getStructFieldOrDefault(config, 'enable_rear_optional_edges', false);
objectiveConfig.rear_box_diagonal_mode = getStructFieldOrDefault(config, 'rear_box_diagonal_mode', 'minimal');
objectiveConfig.diff_reaction_force_N = getStructFieldOrDefault(config, 'diff_reaction_force_N', NaN);
objectiveConfig.diff_reaction_torque_Nm = getStructFieldOrDefault(config, 'diff_reaction_torque_Nm', NaN);
end

function runSummary = buildOptimizationRun(output)
runSummary = struct();
runSummary.exitflag = getStructFieldOrDefault(output, 'exitflag', NaN);
runSummary.generations = getStructFieldOrDefault(output, 'generations', NaN);
runSummary.funccount = getStructFieldOrDefault(output, 'funccount', NaN);
runSummary.elapsed_s = getStructFieldOrDefault(output, 'elapsed_s', NaN);
runSummary.best_history = getStructFieldOrDefault(output, 'history', []);
runSummary.best_stiffness_history = getStructFieldOrDefault(output, 'history_K', []);
runSummary.optimizer_mode = getStructFieldOrDefault(output, 'optimizer_mode', 'mixed_ga');
runSummary.random_seed = getStructFieldOrDefault(output, 'random_seed', []);
runSummary.valid_individual_rate = getStructFieldOrDefault(output, 'valid_individual_rate', NaN);
end

function loadCases = sanitizeLoadCaseResults(caseResults)
loadCases = struct([]);
if isempty(caseResults)
    return;
end
for i = 1:numel(caseResults)
    r = caseResults(i);
    loadCases(i).name = getStructFieldOrDefault(r, 'name', sprintf('case_%d', i));
    loadCases(i).success = getStructFieldOrDefault(r, 'success', false);
    loadCases(i).K_torsion = getStructFieldOrDefault(r, 'K_torsion', NaN);
    loadCases(i).mass_kg = getStructFieldOrDefault(r, 'mass', NaN);
    loadCases(i).max_translation_m = getStructFieldOrDefault(r, 'max_translation_m', NaN);
    loadCases(i).error = getStructFieldOrDefault(r, 'error', '');
end
end

function report = buildRegulationReport(finalResults)
report = struct();
report.compliance_pass = getStructFieldOrDefault(finalResults, 'compliance_pass', false);
report.compliance_penalty = getStructFieldOrDefault(finalResults, 'compliance_penalty', NaN);
report.compliance_report = getStructFieldOrDefault(finalResults, 'compliance_report', struct());
report.geometry_precheck_penalty = getStructFieldOrDefault(finalResults, 'geometry_precheck_penalty', NaN);
report.geometry_precheck_report = getStructFieldOrDefault(finalResults, 'geometry_precheck_report', struct());
end

function stageResult = buildStageResult(opts, frameSection)
stageResult = struct();
stageResult.objective_value = opts.BestObjectiveValue;
stageResult.frame_mass_kg = frameSection.performance.mass_kg;
stageResult.cg_z_m = frameSection.performance.cg_z_m;
stageResult.torsional_stiffness_Nm_per_deg = frameSection.performance.torsional_stiffness_Nm_per_deg;
stageResult.total_length_m = frameSection.performance.total_length_m;
stageResult.num_elements = frameSection.performance.num_elements;
stageResult.selected_pipes = frameSection.selected_pipes;
stageResult.targets = frameSection.targets;
end

function value = extractVectorSlice(values, startIndex, endIndex)
value = [];
if isempty(values) || startIndex > numel(values)
    return;
end
endIndex = min(endIndex, numel(values));
value = values(startIndex:endIndex);
end

function value = getStructFieldOrDefault(s, fieldName, defaultValue)
value = defaultValue;
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
end
end

function out = cellstrOrEmpty(value)
if iscell(value)
    out = cellfun(@char, value, 'UniformOutput', false);
else
    out = {};
end
end
