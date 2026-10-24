function resolution = resolve_authoritative_vehicle_json(projectRoot, varargin)
% RESOLVE_AUTHORITATIVE_VEHICLE_JSON Resolve the SSOT vehicle artifact path for runtime flows.

p = inputParser;
addParameter(p, 'ExplicitPath', '', @(x) ischar(x) || isstring(x));
addParameter(p, 'RequireExportReady', true, @islogical);
addParameter(p, 'RequireSuspensionHardpoints', false, @islogical);
addParameter(p, 'RequiredHardpointCorners', {});
addParameter(p, 'LegacyFallbackPath', '', @(x) ischar(x) || isstring(x));
addParameter(p, 'FallbackOutputPath', '', @(x) ischar(x) || isstring(x));
parse(p, varargin{:});
opts = p.Results;

if nargin < 1 || isempty(projectRoot)
    thisDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(thisDir);
end

resolution = struct();
resolution.path = '';
resolution.exists = false;
resolution.source = 'default_template';
resolution.artifact = struct();
resolution.params = create_vehicle_baseline_template();

explicitPath = char(string(opts.ExplicitPath));
legacyFallbackPath = char(string(opts.LegacyFallbackPath));
fallbackOutputPath = char(string(opts.FallbackOutputPath));

if ~isempty(explicitPath)
    if isfile(explicitPath)
        if explicitArtifactMatchesRequirements(explicitPath, opts)
            resolution.path = explicitPath;
            resolution.source = 'explicit';
        else
            warning('resolve_authoritative_vehicle_json:ExplicitPathRejected', ...
                ['Explicit vehicle artifact did not satisfy the requested contract: %s. ', ...
                 'Falling back to SSOT resolution.'], explicitPath);
        end
    else
        warning('resolve_authoritative_vehicle_json:MissingExplicitPath', ...
            'Explicit vehicle artifact not found: %s. Falling back to SSOT resolution.', explicitPath);
    end
end

if isempty(resolution.path)
    defaultPath = resolve_default_vehicle_json(projectRoot, ...
        'RequireExportReady', opts.RequireExportReady, ...
        'RequireSuspensionHardpoints', opts.RequireSuspensionHardpoints, ...
        'RequiredHardpointCorners', opts.RequiredHardpointCorners);
    if ~isempty(defaultPath)
        resolution.path = defaultPath;
        resolution.source = 'ssot_default';
    end
end

if isempty(resolution.path) && ~isempty(legacyFallbackPath) && isfile(legacyFallbackPath)
    resolution.path = legacyFallbackPath;
    resolution.source = 'legacy_fallback';
end

if isempty(resolution.path) && ~isempty(fallbackOutputPath)
    resolution.path = fallbackOutputPath;
    resolution.source = 'fallback_output';
end

if ~isempty(resolution.path) && isfile(resolution.path)
    resolution.artifact = load_vehicle_artifact(resolution.path);
    resolution.params = resolution.artifact.params;
    resolution.exists = true;
elseif strcmp(resolution.source, 'legacy_fallback')
    resolution.source = 'default_template';
end
end

function tf = explicitArtifactMatchesRequirements(explicitPath, opts)
tf = false;

try
    artifact = load_vehicle_artifact(explicitPath);
catch
    return;
end

if opts.RequireExportReady && ~artifactIsExportReady(artifact)
    return;
end

if opts.RequireSuspensionHardpoints && ~artifactHasSuspensionHardpoints(artifact, opts.RequiredHardpointCorners)
    return;
end

tf = true;
end

function tf = artifactIsExportReady(artifact)
tf = isfield(artifact, 'params') && isstruct(artifact.params) ...
    && isfield(artifact.params, 'mass') ...
    && isfield(artifact.params, 'suspension') ...
    && isfield(artifact.params, 'tire') ...
    && isfield(artifact.params, 'powertrain') ...
    && isfield(artifact.params, 'aero') ...
    && isfield(artifact.params, 'brake');
end

function tf = artifactHasSuspensionHardpoints(artifact, requiredCorners)
tf = false;
if ~isfield(artifact, 'params') || ~isstruct(artifact.params) ...
        || ~isfield(artifact.params, 'suspension') || ~isstruct(artifact.params.suspension) ...
        || ~isfield(artifact.params.suspension, 'kc') || ~isstruct(artifact.params.suspension.kc) ...
        || ~isfield(artifact.params.suspension.kc, 'corners') || ~isstruct(artifact.params.suspension.kc.corners)
    return;
end

corners = artifact.params.suspension.kc.corners;
requiredCorners = cellstr(requiredCorners);
if isempty(requiredCorners)
    requiredCorners = {'FL', 'FR', 'RL', 'RR'};
    for i = 1:numel(requiredCorners)
        wheelName = requiredCorners{i};
        if isfield(corners, wheelName) && isstruct(corners.(wheelName)) ...
                && isfield(corners.(wheelName), 'hardpoints') && isstruct(corners.(wheelName).hardpoints)
            tf = true;
            return;
        end
    end
    return;
end

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
