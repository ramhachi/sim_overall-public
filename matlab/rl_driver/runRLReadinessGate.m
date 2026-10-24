function result = runRLReadinessGate(agentPath, varargin)
    % RUNRLREADINESSGATE End-to-end readiness gate for RL deployment.
    %
    % result = runRLReadinessGate('rl_results/curriculum/final_agent.mat')
    %
    % Steps:
    %   1) Export MATLAB actor for C++
    %   2) Run C++ consistency check (rl_actor_check)
    %   3) Run multi-track readiness validation

    ensure_rl_driver_paths();

    if nargin < 1
        agentPath = fullfile('rl_results', 'curriculum', 'final_agent.mat');
    end

    p = inputParser;
    addParameter(p, 'ActorExportDir', 'rl_actor_export');
    addParameter(p, 'Tracks', {'autocross', 'skidpad', 'circle'});
    addParameter(p, 'Episodes', 5);
    addParameter(p, 'Threshold', 1e-3);
    addParameter(p, 'SaveDir', fullfile('rl_results', 'curriculum'));
    addParameter(p, 'StartServer', true);
    addParameter(p, 'VehicleParams', struct());
    addParameter(p, 'DomainRandStrength', 0.0);
    addParameter(p, 'DomainRandInterval', 1);
    addParameter(p, 'MinCompletionRate', 0.80);
    addParameter(p, 'MaxOffCourseRate', 0.10);
    addParameter(p, 'MaxLapTimeRatio', 1.05);
    addParameter(p, 'PIDBaselineLapTime', struct());
    addParameter(p, 'PublishIfPass', false);
    addParameter(p, 'DeploymentDir', fullfile('rl_results', 'deployment'));
    addParameter(p, 'Metadata', struct());
    parse(p, varargin{:});
    opts = p.Results;

    [agent, ~, agentPath] = loadRLAgentFromFile(agentPath);
    assertAgentObservationDimension(agent, [], 'Readiness gate agent');

    thisDir = fileparts(mfilename('fullpath'));      % .../matlab/rl_driver
    matlabDir = fileparts(thisDir);                  % .../matlab
    projectRoot = fileparts(matlabDir);              % repo root

    actorExportDir = resolvePath(opts.ActorExportDir, thisDir, matlabDir, projectRoot);
    saveDir = resolvePath(opts.SaveDir, thisDir, matlabDir, projectRoot);
    deploymentDir = resolvePath(opts.DeploymentDir, thisDir, matlabDir, projectRoot);
    if ~isfolder(saveDir)
        mkdir(saveDir);
    end

    fprintf('\n=== RL Readiness Gate ===\n');
    fprintf('Agent: %s\n', agentPath);
    fprintf('Publish actor export dir: %s\n', actorExportDir);
    fprintf('Tracks: %s\n', strjoin(opts.Tracks, ', '));
    fprintf('Episodes: %d\n', opts.Episodes);

    % 1) Export actor to a staging dir for parity/readiness checks.
    stagingActorExportDir = fullfile(saveDir, sprintf('actor_export_candidate_%s', datestr(now, 'yyyymmdd_HHMMSSFFF')));
    fprintf('Staging actor export dir: %s\n', stagingActorExportDir);
    exportActorForCpp(agent, stagingActorExportDir, opts.VehicleParams);

    startServerForValidation = opts.StartServer;
    if opts.StartServer
        startRLServer(projectRoot);
        startServerForValidation = false;
    end

    % 2) C++ consistency check
    actorExportDirAbs = char(java.io.File(actorExportDir).getCanonicalPath());
    stagingActorExportDirAbs = char(java.io.File(stagingActorExportDir).getCanonicalPath());
    projectRootAbs = char(java.io.File(projectRoot).getCanonicalPath());
    if ~startsWith(stagingActorExportDirAbs, [projectRootAbs filesep]) && ~strcmp(stagingActorExportDirAbs, projectRootAbs)
        error('Staging actor export dir must be under project root: %s', projectRootAbs);
    end
    actorRel = erase(stagingActorExportDirAbs, [projectRootAbs filesep]);
    actorRel = strrep(actorRel, '\', '/');
    actorDirContainer = ['/workspace/' actorRel];

    composeFile = fullfile(projectRootAbs, 'docker-compose.yml');
    checkCmd = sprintf([ ...
        'docker compose -f "%s" exec -T fsae-sim bash -lc "cd /workspace/build && ' ...
        './cpp/rl_actor_check --actor-dir ''%s'' --test-data ''%s/rl_actor_test_data.csv'' --threshold %.8g"'], ...
        composeFile, actorDirContainer, actorDirContainer, opts.Threshold);
    [checkStatus, checkOutput] = system(checkCmd);
    fprintf('%s\n', checkOutput);
    if checkStatus ~= 0
        error('rl_actor_check failed (status=%d).', checkStatus);
    end

    % 3) Readiness validation
    reportPath = fullfile(saveDir, sprintf('readiness_report_%s.json', datestr(now, 'yyyymmdd_HHMMSS')));
    summary = validateRLReadiness(agent, ...
        'Tracks', opts.Tracks, ...
        'Episodes', opts.Episodes, ...
        'VehicleParams', opts.VehicleParams, ...
        'StartServer', startServerForValidation, ...
        'DomainRandStrength', opts.DomainRandStrength, ...
        'DomainRandInterval', opts.DomainRandInterval, ...
        'MinCompletionRate', opts.MinCompletionRate, ...
        'MaxOffCourseRate', opts.MaxOffCourseRate, ...
        'MaxLapTimeRatio', opts.MaxLapTimeRatio, ...
        'PIDBaselineLapTime', opts.PIDBaselineLapTime, ...
        'SaveJson', reportPath);

    result = struct();
    result.actor_export_dir = stagingActorExportDirAbs;
    result.publish_actor_export_dir = actorExportDirAbs;
    result.report_path = reportPath;
    result.summary = summary;
    result.deployment_published = false;

    canPublish = opts.PublishIfPass && hasPidBaselineForTracks(opts.PIDBaselineLapTime, opts.Tracks);
    if summary.overall_pass && opts.PublishIfPass && ~canPublish
        fprintf('Skipping auto-publish: PIDBaselineLapTime is incomplete for the requested tracks.\n');
    end

    if summary.overall_pass && canPublish
        publishMetadata = merge_nested_structs(struct( ...
            'readiness_report_path', reportPath, ...
            'readiness_summary', summary), opts.Metadata);
        publishResult = publishRLDeploymentCandidate(agentPath, ...
            'ActorExportDir', actorExportDir, ...
            'DeploymentDir', deploymentDir, ...
            'VehicleParams', opts.VehicleParams, ...
            'Metadata', publishMetadata);
        result.deployment_published = true;
        result.deployment = publishResult;
        fprintf('Published deployment candidate: %s\n', publishResult.deployment_agent_path);
    end

    fprintf('\nRL readiness report: %s\n', reportPath);
    fprintf('RL readiness pass: %s\n', mat2str(summary.overall_pass));
end

function resolved = resolvePath(pathIn, rlDir, matlabDir, projectRoot)
    if isAbsolutePath(pathIn)
        resolved = char(pathIn);
        return;
    end
    if isfolder(pathIn) || isfile(pathIn)
        resolved = pathIn;
        return;
    end

    c1 = fullfile(rlDir, pathIn);
    c2 = fullfile(matlabDir, pathIn);
    c3 = fullfile(projectRoot, pathIn);

    if isfolder(fileparts(c1)) || strcmp(fileparts(c1), rlDir)
        resolved = c1;
    elseif isfolder(fileparts(c2)) || strcmp(fileparts(c2), matlabDir)
        resolved = c2;
    else
        resolved = c3;
    end
end

function tf = isAbsolutePath(pathIn)
    pathStr = char(pathIn);
    tf = startsWith(pathStr, filesep) || ~isempty(regexp(pathStr, '^[A-Za-z]:[\\/]', 'once'));
end

function tf = hasPidBaselineForTracks(pidBaselineLapTime, tracks)
tf = true;
for i = 1:numel(tracks)
    trackKey = matlab.lang.makeValidName(tracks{i});
    if ~isfield(pidBaselineLapTime, trackKey)
        tf = false;
        return;
    end
    value = pidBaselineLapTime.(trackKey);
    if isempty(value) || ~isnumeric(value) || ~isscalar(value) || ~(value > 0)
        tf = false;
        return;
    end
end
end
