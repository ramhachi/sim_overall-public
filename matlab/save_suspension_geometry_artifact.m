function result = save_suspension_geometry_artifact(varargin)
% SAVE_SUSPENSION_GEOMETRY_ARTIFACT Persist suspension inverse-analysis results into a unified vehicle artifact JSON.

p = inputParser;
addParameter(p, 'BaseArtifactPath', '', @(x) ischar(x) || isstring(x));
addParameter(p, 'OutputPath', '', @(x) ischar(x) || isstring(x));
addParameter(p, 'InPlace', false, @(x) islogical(x) && isscalar(x));
addParameter(p, 'CurrentParams', struct(), @isstruct);
addParameter(p, 'SuspensionGeometry', struct(), @isstruct);
addParameter(p, 'StageName', 'suspension_geometry', @(x) ischar(x) || isstring(x));
addParameter(p, 'StageOptimizer', 'multistart_fmincon', @(x) ischar(x) || isstring(x));
addParameter(p, 'StageResult', struct(), @isstruct);
parse(p, varargin{:});
opts = p.Results;

baseArtifactPath = char(opts.BaseArtifactPath);
outputPath = char(opts.OutputPath);
inPlace = opts.InPlace;

if isempty(outputPath)
    if isempty(baseArtifactPath)
        error('save_suspension_geometry_artifact:missingOutput', ...
            'OutputPath is required when BaseArtifactPath is empty.');
    end
    if inPlace
        outputPath = baseArtifactPath;
    else
        [baseDir, baseName, ~] = fileparts(baseArtifactPath);
        outputPath = fullfile(baseDir, [baseName, '.optimized.json']);
    end
end

samePath = false;
if ~isempty(baseArtifactPath)
    samePath = paths_refer_to_same_file(baseArtifactPath, outputPath);
end
if samePath && ~inPlace
    error('save_suspension_geometry_artifact:implicitOverwrite', ...
        ['Refusing to overwrite the base artifact ''%s'' by default. ', ...
         'Pass InPlace=true explicitly (a .bak backup is created), or choose a different OutputPath.'], ...
        baseArtifactPath);
end
if inPlace && ~isempty(baseArtifactPath) && ~samePath
    error('save_suspension_geometry_artifact:inPlacePathMismatch', ...
        'InPlace=true requires OutputPath to match BaseArtifactPath (''%s'' vs ''%s'').', ...
        baseArtifactPath, outputPath);
end

artifact = struct();
params = create_vehicle_baseline_template();
if ~isempty(baseArtifactPath) && isfile(baseArtifactPath)
    artifact = load_vehicle_artifact(baseArtifactPath);
    params = merge_nested_structs(params, artifact.params);
end
params = merge_nested_structs(params, opts.CurrentParams);

if ~isempty(fieldnames(opts.SuspensionGeometry))
    if ~isfield(params, 'suspension_geometry') || ~isstruct(params.suspension_geometry)
        params.suspension_geometry = struct();
    end
    params.suspension_geometry = merge_nested_structs(params.suspension_geometry, opts.SuspensionGeometry);
end

params = attach_kc_provenance(params);

stageEntry = struct();
stageEntry.name = char(opts.StageName);
stageEntry.optimizer = char(opts.StageOptimizer);
stageEntry.params = params;
stageEntry.result = opts.StageResult;

artifact = build_vehicle_artifact(params, ...
    'base_artifact', artifact, ...
    'stage_entry', stageEntry);
artifact.environment = get_environment_manifest();

outputDir = fileparts(outputPath);
if ~isempty(outputDir) && ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

backupPath = '';
if samePath && isfile(outputPath)
    backupPath = [outputPath, '.bak'];
    [copied, copyMsg] = copyfile(outputPath, backupPath, 'f');
    if ~copied
        error('save_suspension_geometry_artifact:backupFailed', ...
            'Failed to create backup ''%s'': %s', backupPath, copyMsg);
    end
end

try
    save_vehicle_artifact(outputPath, artifact);
catch save_err
    if ~isempty(backupPath) && isfile(backupPath)
        [restored, restoreMsg] = copyfile(backupPath, outputPath, 'f');
        if ~restored
            warning('save_suspension_geometry_artifact:restoreFailed', ...
                'Save failed and restore from backup failed: %s', restoreMsg);
        end
    end
    rethrow(save_err);
end

result = struct();
result.output_path = outputPath;
result.backup_path = backupPath;
result.in_place = samePath;
result.updated_existing_artifact = ~isempty(baseArtifactPath) && isfile(baseArtifactPath);
result.params = params;
end

function params = attach_kc_provenance(params)
% Rebuild params.suspension.kc.provenance from every corner present (#39), so
% the C++ loader can gate on one block instead of mere key presence.
if ~isfield(params, 'suspension') || ~isfield(params.suspension, 'kc') ...
        || ~isfield(params.suspension.kc, 'corners')
    return;
end
corners = params.suspension.kc.corners;
names = {'FL', 'FR', 'RL', 'RR'};
prov = struct('schema_version', 'kc_provenance/v1', ...
    'updated_at', datestr(now, 'yyyy-mm-dd HH:MM:SS'), 'corners', struct());
missing = {};
allValid = true;
complianceProvisional = false;
for i = 1:numel(names)
    wn = names{i};
    if ~isfield(corners, wn)
        missing{end + 1} = wn; %#ok<AGROW>
        allValid = false;
        continue;
    end
    c = corners.(wn);
    entry = struct('verdict', 'unknown', 'confidence', 'unknown', 'reasons', {{}}, ...
        'model', 'unknown', 'script', 'unknown', 'evaluation_scope', struct());
    if isfield(c, 'quality')
        entry.verdict = getfield_or(c.quality, 'verdict', 'unknown');
        entry.confidence = getfield_or(c.quality, 'confidence', 'unknown');
        entry.reasons = getfield_or(c.quality, 'reasons', {});
    end
    if isfield(c, 'source')
        entry.model = getfield_or(c.source, 'model', 'unknown');
        entry.script = getfield_or(c.source, 'script', 'unknown');
    end
    if isfield(c, 'evaluation_scope')
        entry.evaluation_scope = c.evaluation_scope;
    end
    if isfield(c, 'compliance') && isfield(c.compliance, 'provisional')
        complianceProvisional = complianceProvisional || logical(c.compliance.provisional);
    end
    allValid = allValid && strcmp(entry.verdict, 'valid');
    prov.corners.(wn) = entry;
end
prov.missing_corners = missing;
prov.production_ready = allValid;
prov.compliance_provisional = complianceProvisional;
prov.target_confidence = struct();
for axleKey = {'front', 'rear'}
    a = axleKey{1};
    if isfield(params, 'suspension_geometry') && isfield(params.suspension_geometry, a) ...
            && isfield(params.suspension_geometry.(a), 'target_contract')
        prov.target_confidence.(a) = params.suspension_geometry.(a).target_contract.quality.confidence;
    end
end
params.suspension.kc.provenance = prov;
end

function v = getfield_or(s, name, default)
if isfield(s, name)
    v = s.(name);
else
    v = default;
end
end

function tf = paths_refer_to_same_file(pathA, pathB)
pathA = canonical_path(pathA);
pathB = canonical_path(pathB);
tf = strcmpi(pathA, pathB);
end

function path = canonical_path(path)
try
    path = char(java.io.File(path).getCanonicalPath());
catch
    [dirPart, basePart, extPart] = fileparts(path);
    if isempty(dirPart)
        dirPart = pwd;
    end
    path = fullfile(dirPart, [basePart, extPart]);
end
end
