function result = publishRLDeploymentCandidate(agentOrPath, varargin)
% PUBLISHRLDEPLOYMENTCANDIDATE Publish a selected RL candidate to stable deployment paths.

ensure_rl_driver_paths();

p = inputParser;
addParameter(p, 'ActorExportDir', '');
addParameter(p, 'DeploymentDir', '');
addParameter(p, 'VehicleParams', struct());
addParameter(p, 'Metadata', struct());
parse(p, varargin{:});
opts = p.Results;

thisDir = fileparts(mfilename('fullpath'));
matlabDir = fileparts(thisDir);
projectRoot = fileparts(matlabDir);

if isempty(opts.ActorExportDir)
    actorExportDir = fullfile(thisDir, 'rl_actor_export');
else
    actorExportDir = opts.ActorExportDir;
end
if isempty(opts.DeploymentDir)
    deploymentDir = fullfile(projectRoot, 'rl_results', 'deployment');
else
    deploymentDir = opts.DeploymentDir;
end
if ~exist(deploymentDir, 'dir')
    mkdir(deploymentDir);
end

    [agent, sourcePath, sourceVarName] = resolveDeploymentAgent(agentOrPath);
    assertAgentObservationDimension(agent, [], 'Deployment agent');

    exportActorForCpp(agent, actorExportDir, opts.VehicleParams);

    deploymentAgentPath = fullfile(deploymentDir, 'deployment_agent.mat');
    metadata = struct( ...
        'generated_at', datestr(now, 'yyyy-mm-dd HH:MM:SS'), ...
        'source_agent_path', sourcePath, ...
        'source_variable', sourceVarName, ...
        'deployment_agent_path', deploymentAgentPath, ...
        'actor_export_dir', actorExportDir, ...
        'vehicle_params', opts.VehicleParams);
if ~isempty(opts.Metadata)
    if ~isstruct(opts.Metadata)
        error('Metadata must be a struct.');
    end
    metadata = merge_nested_structs(metadata, opts.Metadata);
end

    metadataPath = fullfile(deploymentDir, 'deployment_metadata.json');
    writePrettyJson(metadataPath, metadata);
    writePrettyJson(fullfile(actorExportDir, 'deployment_metadata.json'), metadata);
    save(deploymentAgentPath, 'agent');

result = struct();
result.deployment_agent_path = deploymentAgentPath;
result.actor_export_dir = actorExportDir;
result.metadata_path = metadataPath;
end

function [agent, sourcePath, sourceVarName] = resolveDeploymentAgent(agentOrPath)
sourcePath = '';
sourceVarName = '';
if ischar(agentOrPath) || isstring(agentOrPath)
    sourcePath = char(agentOrPath);
    [agent, sourceVarName] = loadRLAgentFromFile(sourcePath);
    return;
end

agent = agentOrPath;
end

function writePrettyJson(filePath, value)
dirPath = fileparts(filePath);
if ~isempty(dirPath) && ~exist(dirPath, 'dir')
    mkdir(dirPath);
end
fid = fopen(filePath, 'w');
if fid == -1
    error('Failed to open JSON output path: %s', filePath);
end
fwrite(fid, jsonencode(value, 'PrettyPrint', true), 'char');
fclose(fid);
end
