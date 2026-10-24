function summary = validateRLReadiness(agentOrPath, varargin)
    % VALIDATERLREADINESS Evaluate RL driver readiness for deployment.
    %
    % summary = validateRLReadiness('rl_results/curriculum/final_agent.mat')
    % summary = validateRLReadiness(agent, 'Episodes', 10)
    %
    % Acceptance gates:
    %   1) completion rate
    %   2) off-course rate
    %   3) lap time (median, optionally vs PID baseline ratio)

    p = inputParser;
    addParameter(p, 'Tracks', {'autocross', 'skidpad', 'circle'});
    addParameter(p, 'Episodes', 5);
    addParameter(p, 'VehicleParams', struct());
    addParameter(p, 'StartServer', true);
    addParameter(p, 'DomainRandStrength', 0.0);
    addParameter(p, 'DomainRandInterval', 1);
    addParameter(p, 'MinCompletionRate', 0.80);
    addParameter(p, 'MaxOffCourseRate', 0.10);
    addParameter(p, 'MaxLapTimeRatio', 1.05);
    addParameter(p, 'PIDBaselineLapTime', struct());  % struct with fields: autocross/skidpad/circle
    addParameter(p, 'SaveJson', '');
    parse(p, varargin{:});
    opts = p.Results;

    if ischar(agentOrPath) || isstring(agentOrPath)
        agent = loadRLAgentFromFile(char(agentOrPath));
    else
        agent = agentOrPath;
    end
    assertAgentObservationDimension(agent, [], 'Readiness agent');
    actorModel = getDeterministicActorModel(agent);

    if opts.StartServer
        startRLServer();
    end

    tracks = opts.Tracks;
    if ischar(tracks) || isstring(tracks)
        tracks = cellstr(tracks);
    end

    summary = struct();
    summary.generated_at = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    summary.config = opts;
    summary.tracks = struct();
    summary.overall_pass = true;

    fprintf('\n=== RL Readiness Validation ===\n');
    fprintf('Episodes per track: %d\n', opts.Episodes);
    fprintf('Min completion rate: %.2f\n', opts.MinCompletionRate);
    fprintf('Max off-course rate: %.2f\n', opts.MaxOffCourseRate);
    fprintf('Max lap time ratio: %.2f\n', opts.MaxLapTimeRatio);

    for i = 1:numel(tracks)
        trackName = tracks{i};
        trackKey = matlab.lang.makeValidName(trackName);

        env = FSAEEnvironment(trackName, opts.VehicleParams);
        envCleanup = onCleanup(@() cleanupEnv(env));
        if opts.DomainRandStrength > 0
            env.DomainRandEnabled = true;
            env.DomainRandInterval = opts.DomainRandInterval;
            env.DomainRandStrength = opts.DomainRandStrength;
            env.DomainRandBounds = getVehicleParamBounds();
        end
        completedFlags = false(1, opts.Episodes);
        offCourseFlags = false(1, opts.Episodes);
        lapTimes = nan(1, opts.Episodes);
        terminationReasons = strings(1, opts.Episodes);

        for ep = 1:opts.Episodes
            obs = reset(env);
            stepCount = 0;
            epOffCourse = false;
            done = false;

            while ~done && stepCount < env.MaxEpisodeSteps
                action = predictDeterministicAction(actorModel, obs);
                [obs, ~, done, loggedSignals] = step(env, action);
                stepCount = stepCount + 1;

                distLeft = loggedSignals.distLeft;
                distRight = loggedSignals.distRight;
                edgeMargin = env.OffTrackMargin;

                if distLeft <= edgeMargin || distRight <= edgeMargin
                    epOffCourse = true;
                end
            end

            completedFlags(ep) = env.LapCompleted;
            offCourseFlags(ep) = epOffCourse || strcmp(env.LastTerminationReason, 'offtrack');
            if env.LapCompleted
                lapTimes(ep) = stepCount * env.ControlDt;
            end
            terminationReasons(ep) = string(env.LastTerminationReason);
        end

        delete(envCleanup);

        completionRate = mean(completedFlags);
        offCourseRate = mean(offCourseFlags);
        validLapTimes = lapTimes(~isnan(lapTimes));
        if isempty(validLapTimes)
            medianLap = nan;
        else
            medianLap = median(validLapTimes);
        end

        pidBaseline = nan;
        if isfield(opts.PIDBaselineLapTime, trackKey)
            pidBaseline = opts.PIDBaselineLapTime.(trackKey);
        end

        lapTimeRatio = nan;
        if ~isnan(pidBaseline) && pidBaseline > 0 && ~isnan(medianLap)
            lapTimeRatio = medianLap / pidBaseline;
        end

        completionPass = completionRate >= opts.MinCompletionRate;
        offCoursePass = offCourseRate <= opts.MaxOffCourseRate;
        if ~isnan(lapTimeRatio)
            lapPass = lapTimeRatio <= opts.MaxLapTimeRatio;
        else
            lapPass = ~isnan(medianLap);
        end

        trackPass = completionPass && offCoursePass && lapPass;
        summary.overall_pass = summary.overall_pass && trackPass;

        trackSummary = struct();
        trackSummary.track = trackName;
        trackSummary.episodes = opts.Episodes;
        trackSummary.completion_rate = completionRate;
        trackSummary.off_course_rate = offCourseRate;
        trackSummary.median_lap_time = medianLap;
        trackSummary.pid_baseline_lap_time = pidBaseline;
        trackSummary.lap_time_ratio = lapTimeRatio;
        trackSummary.pass = trackPass;
        trackSummary.pass_detail = struct( ...
            'completion', completionPass, ...
            'off_course', offCoursePass, ...
            'lap_time', lapPass);
        trackSummary.termination_reason_counts = countReasons(terminationReasons);

        summary.tracks.(trackKey) = trackSummary;

        fprintf('\n[%s]\n', trackName);
        fprintf('  completion_rate: %.3f\n', completionRate);
        fprintf('  off_course_rate: %.3f\n', offCourseRate);
        if isnan(medianLap)
            fprintf('  median_lap_time: NaN\n');
        else
            fprintf('  median_lap_time: %.3f s\n', medianLap);
        end
        if ~isnan(lapTimeRatio)
            fprintf('  lap_time_ratio_vs_pid: %.3f\n', lapTimeRatio);
        else
            fprintf('  lap_time_ratio_vs_pid: N/A (no baseline)\n');
        end
        fprintf('  PASS: %s\n', mat2str(trackPass));
    end

    fprintf('\nOverall PASS: %s\n', mat2str(summary.overall_pass));

    if ~isempty(opts.SaveJson)
        fid = fopen(opts.SaveJson, 'w');
        if fid == -1
            error('Failed to open SaveJson path: %s', opts.SaveJson);
        end
        fwrite(fid, jsonencode(summary, 'PrettyPrint', true), 'char');
        fclose(fid);
        fprintf('Saved readiness report: %s\n', opts.SaveJson);
    end
end

function cleanupEnv(env)
    if ~isempty(env) && isvalid(env)
        delete(env);
    end
end

function counts = countReasons(reasons)
    counts = struct();
    uniqueReasons = unique(reasons);
    for i = 1:numel(uniqueReasons)
        reason = char(uniqueReasons(i));
        if isempty(reason)
            continue;
        end
        key = matlab.lang.makeValidName(reason);
        counts.(key) = sum(reasons == uniqueReasons(i));
    end
end
