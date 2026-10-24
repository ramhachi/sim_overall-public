function candidates = find_vehicle_json_candidates(projectRoot, varargin)
% FIND_VEHICLE_JSON_CANDIDATES Discover vehicle JSON files relevant to sim_overall.

p = inputParser;
addParameter(p, 'RequireExportReady', false);
addParameter(p, 'RequireSuspensionHardpoints', false);
addParameter(p, 'RequiredHardpointCorners', {});
parse(p, varargin{:});
opts = p.Results;

if nargin < 1 || isempty(projectRoot)
    thisDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(thisDir);
end

searchRoots = {
    fullfile(projectRoot, 'matlab')
    fullfile(projectRoot, 'matlab', 'parameter_presets')
    fullfile(projectRoot, 'output', 'runtime')
    };

seen = containers.Map('KeyType', 'char', 'ValueType', 'logical');
candidates = struct('path', {}, 'label', {}, 'name', {}, 'group', {}, ...
    'datenum', {}, 'modified_at', {}, 'is_optimized', {}, ...
    'has_suspension_hardpoints', {}, 'summary', {}, 'params', {});

for rootIdx = 1:numel(searchRoots)
    rootDir = searchRoots{rootIdx};
    if ~isfolder(rootDir)
        continue;
    end

    jsonFiles = dir(fullfile(rootDir, '**', '*.json'));
    for fileIdx = 1:numel(jsonFiles)
        filePath = fullfile(jsonFiles(fileIdx).folder, jsonFiles(fileIdx).name);
        if isTransientRuntimeParamsFile(filePath)
            continue;
        end
        if isKey(seen, filePath)
            continue;
        end
        seen(filePath) = true;

        try
            artifact = load_vehicle_artifact(filePath);
        catch
            continue;
        end

        rawParams = artifact.params;
        isFullVehicle = isVehicleParamsStruct(rawParams);
        isOptimized = detectOptimizedCandidate(artifact, jsonFiles(fileIdx).name);
        if opts.RequireExportReady
            if ~isFullVehicle
                continue;
            end
        elseif ~(isFullVehicle || looksLikeVehicleParamFragment(rawParams) || isOptimized)
            continue;
        end

        mergedParams = merge_nested_structs(create_vehicle_baseline_template(), artifact.params);
        hasSuspensionHardpoints = candidateHasSuspensionHardpoints(artifact, opts.RequiredHardpointCorners);

        if opts.RequireSuspensionHardpoints && ~hasSuspensionHardpoints
            continue;
        end

        entry = struct();
        entry.path = filePath;
        entry.label = toProjectRelativePath(filePath, projectRoot);
        entry.name = jsonFiles(fileIdx).name;
        entry.group = detectCandidateGroup(filePath, projectRoot);
        entry.datenum = jsonFiles(fileIdx).datenum;
        entry.modified_at = datestr(jsonFiles(fileIdx).datenum, 'yyyy-mm-dd HH:MM:SS');
        entry.is_optimized = isOptimized;
        entry.has_suspension_hardpoints = hasSuspensionHardpoints;
        entry.summary = buildCandidateSummary(artifact, mergedParams);
        entry.params = mergedParams;
        candidates(end+1) = entry; %#ok<AGROW>
    end
end

if isempty(candidates)
    return;
end

sortRows = zeros(numel(candidates), 3);
for i = 1:numel(candidates)
    sortRows(i, 1) = ~candidates(i).is_optimized;
    sortRows(i, 2) = groupPriority(candidates(i).group);
    sortRows(i, 3) = -candidates(i).datenum;
end

[~, order] = sortrows(sortRows, [1, 2, 3]);
candidates = candidates(order);
end

function tf = isVehicleParamsStruct(params)
tf = isstruct(params) && isfield(params, 'mass') && isfield(params, 'suspension') ...
    && isfield(params, 'tire') && isfield(params, 'powertrain') ...
    && isfield(params, 'aero') && isfield(params, 'brake');
end

function tf = looksLikeVehicleParamFragment(params)
tf = isstruct(params) && any(isfield(params, ...
    {'mass', 'suspension', 'tire', 'powertrain', 'aero', 'brake', 'steering', 'drs'}));
end

function group = detectCandidateGroup(filePath, projectRoot)
if startsWith(filePath, fullfile(projectRoot, 'matlab', 'parameter_presets'))
    group = 'preset';
elseif startsWith(filePath, fullfile(projectRoot, 'output', 'runtime'))
    group = 'runtime';
else
    group = 'matlab';
end
end

function priority = groupPriority(group)
switch group
    case 'matlab'
        priority = 0;
    case 'runtime'
        priority = 1;
    case 'preset'
        priority = 2;
    otherwise
        priority = 3;
end
end

function tf = detectOptimizedCandidate(artifact, fileName)
tf = startsWith(fileName, 'optimized_', 'IgnoreCase', true) ...
    || contains(fileName, 'optimized', 'IgnoreCase', true) ...
    || isfield(artifact, 'design_vector') ...
    || isfield(artifact, 'lap_time') ...
    || (isfield(artifact, 'params') && isstruct(artifact.params) ...
        && isfield(artifact.params, 'lap_time_prediction'));
end

function summary = buildCandidateSummary(artifact, params)
parts = {};

if isfield(artifact, 'event') && ~isempty(artifact.event)
    parts{end+1} = sprintf('event=%s', char(string(artifact.event))); %#ok<AGROW>
end

if isfield(params, 'mass') && isfield(params.mass, 'm_total')
    parts{end+1} = sprintf('mass=%.1fkg', params.mass.m_total); %#ok<AGROW>
end

if isfield(params, 'mass') && isfield(params.mass, 'wheelbase')
    parts{end+1} = sprintf('wb=%.3fm', params.mass.wheelbase); %#ok<AGROW>
end

if isfield(params, 'aero') && isfield(params.aero, 'CL_A') && isfield(params.aero, 'CD_A')
    parts{end+1} = sprintf('CL_A=%.3f CD_A=%.3f', params.aero.CL_A, params.aero.CD_A); %#ok<AGROW>
end

if isfield(params, 'suspension') && isfield(params.suspension, 'K_chassis')
    parts{end+1} = sprintf('K=%.0fNm/deg', params.suspension.K_chassis); %#ok<AGROW>
end

if candidateHasSuspensionHardpoints(artifact)
    parts{end+1} = 'suspension=hardpoints'; %#ok<AGROW>
end

if isfield(artifact, 'params') && isfield(artifact.params, 'lap_time_prediction')
    parts{end+1} = sprintf('lap=%.3fs', artifact.params.lap_time_prediction); %#ok<AGROW>
elseif isfield(artifact, 'lap_time')
    parts{end+1} = sprintf('lap=%.3fs', artifact.lap_time); %#ok<AGROW>
end

summary = strjoin(parts, ', ');
if isempty(summary)
    summary = 'vehicle params';
end
end

function tf = candidateHasSuspensionHardpoints(artifact, requiredCorners)
if nargin < 2
    requiredCorners = {};
end

tf = false;
if ~isfield(artifact, 'params') || ~isstruct(artifact.params) ...
        || ~isfield(artifact.params, 'suspension') || ~isstruct(artifact.params.suspension) ...
        || ~isfield(artifact.params.suspension, 'kc') || ~isstruct(artifact.params.suspension.kc) ...
        || ~isfield(artifact.params.suspension.kc, 'corners') || ~isstruct(artifact.params.suspension.kc.corners)
    return;
end

corners = artifact.params.suspension.kc.corners;
if isempty(requiredCorners)
    wheelNames = {'FL', 'FR', 'RL', 'RR'};
    for i = 1:numel(wheelNames)
        wheelName = wheelNames{i};
        if isfield(corners, wheelName) && isstruct(corners.(wheelName)) ...
                && isfield(corners.(wheelName), 'hardpoints') && isstruct(corners.(wheelName).hardpoints)
            tf = true;
            return;
        end
    end
else
    requiredCorners = cellstr(requiredCorners);
    tf = true;
    for i = 1:numel(requiredCorners)
        wheelName = char(string(requiredCorners{i}));
        if ~(isfield(corners, wheelName) && isstruct(corners.(wheelName)) ...
                && isfield(corners.(wheelName), 'hardpoints') && isstruct(corners.(wheelName).hardpoints))
            tf = false;
            return;
        end
    end
end
end

function tf = isTransientRuntimeParamsFile(filePath)
[parentDir, baseName, ext] = fileparts(filePath);
tf = strcmp(ext, '.json') ...
    && contains(parentDir, [filesep 'output' filesep 'runtime']) ...
    && any(strcmp(baseName, {'params_run', 'params_drs_run'}));
end

function relPath = toProjectRelativePath(filePath, projectRoot)
prefix = [projectRoot filesep];
if startsWith(filePath, prefix)
    relPath = filePath(numel(prefix)+1:end);
else
    relPath = filePath;
end
end
