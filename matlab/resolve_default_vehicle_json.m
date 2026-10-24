function jsonPath = resolve_default_vehicle_json(projectRoot, varargin)
% RESOLVE_DEFAULT_VEHICLE_JSON Return the highest-priority vehicle JSON candidate.

p = inputParser;
addParameter(p, 'RequireExportReady', true);
addParameter(p, 'RequireSuspensionHardpoints', false);
addParameter(p, 'RequiredHardpointCorners', {});
parse(p, varargin{:});
opts = p.Results;

if nargin < 1 || isempty(projectRoot)
    thisDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(thisDir);
end

candidates = find_vehicle_json_candidates(projectRoot, ...
    'RequireExportReady', opts.RequireExportReady, ...
    'RequireSuspensionHardpoints', opts.RequireSuspensionHardpoints, ...
    'RequiredHardpointCorners', opts.RequiredHardpointCorners);

if isempty(candidates)
    jsonPath = '';
else
    jsonPath = candidates(1).path;
end
end
