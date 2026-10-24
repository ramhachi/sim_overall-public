function [agent, variableName, resolvedPath] = loadRLAgentFromFile(agentPath)
% LOADRLAGENTFROMFILE Load an RL agent from agent.mat / saved checkpoint files.

resolvedPath = resolveAgentPath(agentPath);

data = load(resolvedPath);
preferredNames = {'agent', 'saved_agent'};
for i = 1:numel(preferredNames)
    if isfield(data, preferredNames{i})
        variableName = preferredNames{i};
        agent = data.(preferredNames{i});
        return;
    end
end

names = fieldnames(data);
if isempty(names)
    error('Agent file does not contain a loadable variable: %s', resolvedPath);
end

variableName = names{1};
agent = data.(variableName);
end

function resolvedPath = resolveAgentPath(agentPath)
resolvedPath = char(agentPath);
if isfile(resolvedPath)
    return;
end

thisDir = fileparts(mfilename('fullpath'));
matlabDir = fileparts(thisDir);
projectRoot = fileparts(matlabDir);

candidates = {
    resolvedPath
    fullfile(thisDir, resolvedPath)
    fullfile(matlabDir, resolvedPath)
    fullfile(projectRoot, resolvedPath)
    };

for i = 1:numel(candidates)
    if isfile(candidates{i})
        resolvedPath = candidates{i};
        return;
    end
end

error('Agent file not found: %s', agentPath);
end
