function suspension_file = resolve_suspension_geometry_source(project_root, preferred_file, varargin)
% RESOLVE_SUSPENSION_GEOMETRY_SOURCE Resolve the authoritative artifact with suspension hardpoints.

if nargin < 2
    preferred_file = '';
end

p = inputParser;
addParameter(p, 'ErrorIfMissing', false, @(x) islogical(x) || isnumeric(x));
parse(p, varargin{:});
error_if_missing = logical(p.Results.ErrorIfMissing);

if exist('find_vehicle_json_candidates', 'file') ~= 2
    addpath(fullfile(project_root, 'matlab'));
end

preferred_file = char(string(preferred_file));

if has_artifact_hardpoints(preferred_file)
    suspension_file = preferred_file;
    return;
end

candidates = find_vehicle_json_candidates(project_root, ...
    'RequireExportReady', true, ...
    'RequireSuspensionHardpoints', true, ...
    'RequiredHardpointCorners', {'FL', 'RL'});
if ~isempty(candidates)
    suspension_file = candidates(1).path;
    return;
end

suspension_file = '';
msg = sprintf('No vehicle artifact with suspension hardpoints was found under %s. Run EasyStart [8] first.', project_root);
if error_if_missing
    error('resolve_suspension_geometry_source:MissingGeometry', '%s', msg);
end
warning('resolve_suspension_geometry_source:MissingGeometry', '%s', msg);
end

function tf = has_artifact_hardpoints(json_path)
tf = false;
if strlength(string(json_path)) == 0 || ~isfile(json_path)
    return;
end

try
    artifact = load_vehicle_artifact(json_path);
    tf = isfield(artifact.params, 'suspension') && isfield(artifact.params.suspension, 'kc') ...
        && isfield(artifact.params.suspension.kc, 'corners') ...
        && ( ...
            (isfield(artifact.params.suspension.kc.corners, 'FL') && isfield(artifact.params.suspension.kc.corners.FL, 'hardpoints')) ...
            || ...
            (isfield(artifact.params.suspension.kc.corners, 'RL') && isfield(artifact.params.suspension.kc.corners.RL, 'hardpoints')) ...
        );
catch
    tf = false;
end
end
