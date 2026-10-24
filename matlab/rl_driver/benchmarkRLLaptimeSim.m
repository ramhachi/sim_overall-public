function results = benchmarkRLLaptimeSim(varargin)
% BENCHMARKRLLAPTIMESIM Benchmark RL laptime simulation wall-clock and lap times.
%
% results = benchmarkRLLaptimeSim()
% results = benchmarkRLLaptimeSim('Tracks', {'autocross', 'skidpad'})
% results = benchmarkRLLaptimeSim('ActorDir', 'rl_results/.../final_actor_export')
%
% Measures:
%   - lap_time reported by laptime_sim
%   - lap_completed
%   - wall_clock_sec measured around the simulator invocation
%   - benchmark_command / benchmark_mode for traceability

ensure_rl_driver_paths();

p = inputParser;
addParameter(p, 'Tracks', {'autocross', 'skidpad'});
addParameter(p, 'ActorDir', '');
addParameter(p, 'ProjectRoot', '');
addParameter(p, 'Rebuild', false, @(x) islogical(x) && isscalar(x));
addParameter(p, 'VehicleParams', struct(), @isstruct);
addParameter(p, 'SaveJson', '', @(x) ischar(x) || isstring(x));
parse(p, varargin{:});
opts = p.Results;

projectRoot = resolveProjectRoot(opts.ProjectRoot);
actorDirHost = resolveBenchmarkActorDir(opts.ActorDir, projectRoot);
actorDirContainer = hostPathToContainer(actorDirHost, projectRoot);
vehicleParams = resolve_rl_vehicle_params(opts.VehicleParams);
composeFile = fullfile(projectRoot, 'docker-compose.yml');

if opts.Rebuild
    buildCmd = sprintf([ ...
        'docker compose -f "%s" exec -T fsae-sim bash -lc ' ...
        '"cd /workspace/build && cmake --build . --target laptime_sim"'], ...
        composeFile);
    [buildStatus, buildOutput] = system(buildCmd);
    if buildStatus ~= 0
        error('Failed to build laptime_sim:\n%s', buildOutput);
    end
end

tracks = normalizeTracks(opts.Tracks);
results = repmat(struct( ...
    'track', '', ...
    'lap_time', NaN, ...
    'lap_completed', false, ...
    'distance_traveled', NaN, ...
    'max_lateral_error', NaN, ...
    'avg_lateral_error', NaN, ...
    'wall_clock_sec', NaN, ...
    'actor_dir', actorDirHost, ...
    'benchmark_mode', '', ...
    'benchmark_command', ''), 1, numel(tracks));

fprintf('=== RL Laptime Benchmark ===\n');
fprintf('Actor dir: %s\n', actorDirHost);

for i = 1:numel(tracks)
    trackName = tracks{i};
    [paramsPathHost, paramsPathContainer] = writeBenchmarkParamsFile(projectRoot, vehicleParams, trackName);
    [status, cmdout, wallClockSec, usedMode, usedCmd] = runBenchmarkCommand( ...
        composeFile, projectRoot, trackName, actorDirHost, actorDirContainer, paramsPathHost, paramsPathContainer);
    deleteParamsFile(paramsPathHost);
    if status ~= 0
        error('Benchmark failed for %s:\n%s', trackName, cmdout);
    end

    payload = extractJsonPayload(cmdout);
    result = jsondecode(payload);
    result.track = trackName;
    result.wall_clock_sec = wallClockSec;
    result.actor_dir = actorDirHost;
    result.benchmark_mode = usedMode;
    result.benchmark_command = usedCmd;
    results(i) = result;

    fprintf('[%s] lap_completed=%d, lap_time=%.3f s, wall_clock=%.3f s\n', ...
        trackName, result.lap_completed, result.lap_time, result.wall_clock_sec);
end

savePath = char(opts.SaveJson);
if isempty(savePath)
    saveDir = fullfile(projectRoot, 'output', 'runtime');
    if ~exist(saveDir, 'dir')
        mkdir(saveDir);
    end
    savePath = fullfile(saveDir, sprintf('rl_laptime_benchmark_%s.json', datestr(now, 'yyyymmdd_HHMMSS')));
elseif ~startsWith(savePath, filesep) && isempty(regexp(savePath, '^[A-Za-z]:[\\/]', 'once'))
    savePath = fullfile(projectRoot, savePath);
end
writeBenchmarkJson(savePath, results);
fprintf('Saved benchmark JSON: %s\n', savePath);
end

function [status, cmdout, wallClockSec, usedMode, usedCmd] = runBenchmarkCommand( ...
    composeFile, projectRoot, trackName, actorDirHost, actorDirContainer, paramsPathHost, paramsPathContainer)
dockerCmd = sprintf([ ...
    'docker compose -f "%s" exec -T fsae-sim bash -lc ' ...
    '"cd /workspace/build && ./cpp/laptime_sim --track %s --driver rl ' ...
    '--rl-actor-dir ''%s'' --params_file ''%s'' --output_json"'], ...
    composeFile, trackName, actorDirContainer, paramsPathContainer);

tStart = tic;
[status, cmdout] = system(dockerCmd);
wallClockSec = toc(tStart);
usedMode = 'docker-exec';
usedCmd = dockerCmd;
if status == 0
    return;
end

fallbackBinaries = { ...
    fullfile(projectRoot, 'build-local', 'laptime_sim') ...
    fullfile(projectRoot, 'build', 'cpp', 'laptime_sim') ...
    };
for i = 1:numel(fallbackBinaries)
    if ~isfile(fallbackBinaries{i})
        continue;
    end
    localCmd = sprintf('"%s" --track %s --driver rl --rl-actor-dir "%s" --params_file "%s" --output_json', ...
        fallbackBinaries{i}, trackName, actorDirHost, paramsPathHost);
    tStart = tic;
    [status, cmdout] = system(localCmd);
    wallClockSec = toc(tStart);
    usedMode = 'host-binary';
    usedCmd = localCmd;
    if status == 0
        return;
    end
end
end

function projectRoot = resolveProjectRoot(projectRootArg)
if ~isempty(projectRootArg)
    projectRoot = char(projectRootArg);
    return;
end

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));
end

function actorDirHost = resolveBenchmarkActorDir(actorDirArg, projectRoot)
if ~isempty(actorDirArg)
    actorDirHost = resolvePathUnderProject(char(actorDirArg), projectRoot);
else
    actorDirHost = resolve_default_rl_actor_export_dir(projectRoot);
    if isempty(actorDirHost)
        error('No RL actor export directory found.');
    end
end

requiredFiles = {'rl_actor_weights.txt', 'rl_normalization_params.txt', 'rl_actor_test_data.csv'};
for i = 1:numel(requiredFiles)
    requiredPath = fullfile(actorDirHost, requiredFiles{i});
    if ~isfile(requiredPath)
        error('Missing RL actor export file: %s', requiredPath);
    end
end
end

function pathOut = resolvePathUnderProject(pathIn, projectRoot)
if startsWith(pathIn, filesep) || ~isempty(regexp(pathIn, '^[A-Za-z]:[\\/]', 'once'))
    pathOut = pathIn;
else
    pathOut = fullfile(projectRoot, pathIn);
end
end

function actorDirContainer = hostPathToContainer(actorDirHost, projectRoot)
actorDirHost = char(java.io.File(actorDirHost).getCanonicalPath());
projectRoot = char(java.io.File(projectRoot).getCanonicalPath());
if ~startsWith(actorDirHost, [projectRoot filesep]) && ~strcmp(actorDirHost, projectRoot)
    error('ActorDir must be under project root: %s', projectRoot);
end

relativePath = erase(actorDirHost, [projectRoot filesep]);
relativePath = strrep(relativePath, '\', '/');
actorDirContainer = ['/workspace/' relativePath];
end

function tracks = normalizeTracks(trackValue)
if ischar(trackValue) || (isstring(trackValue) && isscalar(trackValue))
    tracks = {char(trackValue)};
elseif isstring(trackValue)
    tracks = cellstr(trackValue(:));
elseif iscell(trackValue)
    tracks = cellfun(@char, trackValue, 'UniformOutput', false);
else
    error('Tracks must be a char, string, or cell array.');
end
end

function payload = extractJsonPayload(cmdout)
jsonStart = strfind(cmdout, '{');
jsonEnd = strfind(cmdout, '}');
if isempty(jsonStart) || isempty(jsonEnd)
    error('No JSON payload found in command output:\n%s', cmdout);
end

payload = cmdout(jsonStart(1):jsonEnd(end));
end

function [paramsPathHost, paramsPathContainer] = writeBenchmarkParamsFile(projectRoot, vehicleParams, trackName)
runtimeDirHost = fullfile(projectRoot, 'output', 'runtime');
if ~exist(runtimeDirHost, 'dir')
    mkdir(runtimeDirHost);
end

fileName = sprintf('rl_benchmark_params_%s_%s.json', trackName, datestr(now, 'yyyymmddHHMMSSFFF'));
paramsPathHost = fullfile(runtimeDirHost, fileName);
paramsPathContainer = sprintf('/workspace/output/runtime/%s', fileName);

fid = fopen(paramsPathHost, 'w');
if fid == -1
    error('Failed to create benchmark params file: %s', paramsPathHost);
end
cleanup = onCleanup(@() fclose(fid));
fwrite(fid, jsonencode(vehicleParams), 'char');
clear cleanup;
end

function deleteParamsFile(paramsPathHost)
if exist(paramsPathHost, 'file')
    delete(paramsPathHost);
end
end

function writeBenchmarkJson(savePath, results)
saveDir = fileparts(savePath);
if ~isempty(saveDir) && ~exist(saveDir, 'dir')
    mkdir(saveDir);
end

fid = fopen(savePath, 'w');
if fid == -1
    error('Failed to write benchmark JSON: %s', savePath);
end
cleanup = onCleanup(@() fclose(fid));
fwrite(fid, jsonencode(results, 'PrettyPrint', true), 'char');
clear cleanup;
end
