function artifact = build_vehicle_artifact(params, varargin)
% build_vehicle_artifact - 車両諸元と最適化結果を単一 artifact JSON 用 struct にまとめる

p = inputParser;
addRequired(p, 'params', @isstruct);
addParameter(p, 'base_artifact', struct(), @isstruct);
addParameter(p, 'stage_entry', struct(), @isstruct);
addParameter(p, 'event_name', '', @(x) ischar(x) || isstring(x));
addParameter(p, 'event_result', struct(), @isstruct);
parse(p, params, varargin{:});

artifact = p.Results.base_artifact;
if isempty(fieldnames(artifact))
    artifact = struct();
end

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

artifact.params = params;

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

stage_entry = p.Results.stage_entry;
if ~isempty(fieldnames(stage_entry))
    if ~isfield(stage_entry, 'params') || ~isstruct(stage_entry.params)
        stage_entry.params = params;
    end

    artifact.optimization.stages{end + 1} = stage_entry;
end

if ~isfield(artifact, 'performance') || ~isstruct(artifact.performance)
    artifact.performance = struct();
end
if ~isfield(artifact.performance, 'events') || ~isstruct(artifact.performance.events)
    artifact.performance.events = struct();
end

event_name = p.Results.event_name;
if isstring(event_name)
    event_name = char(event_name);
end
event_name = strtrim(event_name);
event_result = p.Results.event_result;
if ~isempty(event_name) && ~isempty(fieldnames(event_result))
    event_key = matlab.lang.makeValidName(event_name);
    if ~isfield(event_result, 'name') || isempty(event_result.name)
        event_result.name = event_name;
    end
    artifact.performance.events.(event_key) = event_result;
end

end
