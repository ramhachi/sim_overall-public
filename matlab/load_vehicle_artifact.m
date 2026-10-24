function artifact = load_vehicle_artifact(json_path)
% load_vehicle_artifact - unified artifact または legacy params JSON を読み込む

if ~exist(json_path, 'file')
    error('Vehicle artifact file not found: %s', json_path);
end

raw_text = fileread(json_path);
decoded = jsondecode(raw_text);
params = extract_vehicle_params_from_artifact(decoded);

artifact = decoded;
artifact.params = params;

if ~isfield(artifact, 'schema_version') || isempty(artifact.schema_version)
    artifact.schema_version = 'vehicle_artifact/v1';
end

if ~isfield(artifact, 'generated_by') || ~isstruct(artifact.generated_by)
    artifact.generated_by = struct();
end
if ~isfield(artifact.generated_by, 'tool') || isempty(artifact.generated_by.tool)
    artifact.generated_by.tool = 'sim_overall';
end
if ~isfield(artifact.generated_by, 'physics_core') || isempty(artifact.generated_by.physics_core)
    artifact.generated_by.physics_core = 'sim_overall/cpp';
end
if ~isfield(artifact.generated_by, 'source_of_truth') || isempty(artifact.generated_by.source_of_truth)
    artifact.generated_by.source_of_truth = 'FOPT/sim/cpp';
end

if ~isfield(artifact, 'optimization') || ~isstruct(artifact.optimization)
    artifact.optimization = struct();
end
if ~isfield(artifact.optimization, 'stages') || isempty(artifact.optimization.stages)
    artifact.optimization.stages = {};
elseif isstruct(artifact.optimization.stages)
    artifact.optimization.stages = num2cell(artifact.optimization.stages);
elseif ~iscell(artifact.optimization.stages)
    artifact.optimization.stages = {artifact.optimization.stages};
end

if ~isfield(artifact, 'performance') || ~isstruct(artifact.performance)
    artifact.performance = struct();
end
if ~isfield(artifact.performance, 'events') || ~isstruct(artifact.performance.events)
    artifact.performance.events = struct();
end

end
