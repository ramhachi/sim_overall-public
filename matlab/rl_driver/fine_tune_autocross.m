function result = fine_tune_autocross(config)
% FINE_TUNE_AUTOCROSS Fine-tune a torque-steering autocross RL driver.
%
% result = fine_tune_autocross()
% result = fine_tune_autocross(struct('max_episodes', 50))
%
% The default target is a fresh v7 output directory. If v7 already contains
% Agent<N>.mat checkpoints, training resumes from the highest episode number.

ensure_rl_driver_paths();

if nargin < 1 || isempty(config)
    config = struct();
end
if ~isstruct(config)
    error('fine_tune_autocross:InvalidConfig', 'config must be a struct.');
end

config = normalize_config(config);
if config.reset_output && isfolder(config.output_dir)
    rmdir(config.output_dir, 's');
end
ensure_dir(config.output_dir);

result = initial_result(config);
write_json(config.run_manifest_path, result);

try
    if config.start_server
        startRLServer(config.project_root);
    end

    [agent_file, resume_episode] = resolve_start_agent(config);
    fprintf('Loading fine-tune source agent: %s\n', agent_file);
    [agent, agent_variable_name, resolved_agent_file] = loadRLAgentFromFile(agent_file);

    vehicle_params = resolve_rl_vehicle_params();
    vehicle_params.steering.torque_mode = true;
    vehicle_params.steering.T_driver_max = config.driver_torque_max;
    vehicle_params.steering.T_driver_rate_max = config.driver_torque_rate_max;
    fprintf('Target steering driver torque: %.1f Nm, rate limit: %.1f Nm/s (0 = unlimited)\n', ...
        vehicle_params.steering.T_driver_max, vehicle_params.steering.T_driver_rate_max);

    env = FSAEEnvironment(config.track, vehicle_params);
    cleanup_env = onCleanup(@() cleanup_environment(env));
    configure_environment(env, config, resume_episode);

    train_opts = rlTrainingOptions( ...
        'MaxEpisodes', config.max_episodes, ...
        'MaxStepsPerEpisode', env.MaxEpisodeSteps, ...
        'ScoreAveragingWindowLength', config.score_window, ...
        'StopTrainingCriteria', 'AverageReward', ...
        'StopTrainingValue', config.stop_training_value, ...
        'SaveAgentCriteria', 'EpisodeFrequency', ...
        'SaveAgentValue', config.save_agent_frequency, ...
        'SaveAgentDirectory', config.output_dir, ...
        'Plots', config.plots, ...
        'Verbose', config.verbose);

    result.status = 'training';
    result.source_agent_file = resolved_agent_file;
    result.source_agent_variable = agent_variable_name;
    result.resume_episode = resume_episode;
    result.vehicle_params = vehicle_params;
    write_json(config.run_manifest_path, result);

    fprintf('=== Starting autocross fine-tune: %s ===\n', config.run_id);
    trainingStats = train(agent, env, train_opts);

    save(fullfile(config.output_dir, 'trained_agent.mat'), 'agent');
    save(fullfile(config.output_dir, 'training_stats.mat'), 'trainingStats');

    latest_checkpoint = findLatestAgentByEpisode(config.output_dir);
    result.latest_checkpoint = latest_checkpoint;
    result.status = 'success';
    result.completed_at_utc = utc_now();

    if config.export_final
        exportActorForCpp(agent, fullfile(config.output_dir, 'final_actor_export'), vehicle_params);
        exportActorForCpp(agent, fullfile(config.project_root, 'rl_actor_export'), vehicle_params);
    end

    if config.benchmark_after_training
        benchmark_config = struct();
        benchmark_config.project_root = config.project_root;
        benchmark_config.run_id = [config.run_id '_post_train'];
        benchmark_config.checkpoint_dir = config.output_dir;
        benchmark_config.max_checkpoints = config.benchmark_max_checkpoints;
        benchmark_config.sim_time = config.benchmark_sim_time;
        benchmark_config.driver_torque_max = config.driver_torque_max;
        benchmark_config.driver_torque_rate_max = config.driver_torque_rate_max;
        benchmark_config.rl_steer_torque_rate_limit = config.rl_steer_torque_rate_limit;
        result.benchmark = benchmarkRLCheckpoints(benchmark_config);
    end

    write_json(config.run_manifest_path, result);
    fprintf('=== Fine-tuning complete: %s ===\n', config.output_dir);
catch ME
    result.status = 'failed';
    result.failed_at_utc = utc_now();
    result.error_identifier = ME.identifier;
    result.error_message = ME.message;
    result.error_report = getReport(ME, 'extended', 'hyperlinks', 'off');
    result.latest_checkpoint = findLatestAgentByEpisode(config.output_dir);
    write_json(config.crash_manifest_path, result);
    write_json(config.run_manifest_path, result);
    fprintf(2, 'Fine-tuning failed. Crash manifest: %s\n', config.crash_manifest_path);
    if config.rethrow_on_error
        rethrow(ME);
    end
end
end

function config = normalize_config(config)
if ~isfield(config, 'project_root') || isempty(config.project_root)
    config.project_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
else
    config.project_root = char(string(config.project_root));
end
config.project_root = char(java.io.File(config.project_root).getCanonicalPath());

if ~isfield(config, 'run_id') || isempty(config.run_id)
    config.run_id = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
end
if ~isfield(config, 'output_dir') || isempty(config.output_dir)
    config.output_dir = fullfile(config.project_root, 'rl_results', ...
        'autocross_winged_flexible_ft_v7_smooth_recovery_40Nm');
else
    config.output_dir = resolve_project_path(config.project_root, config.output_dir);
end
if ~isfield(config, 'track') || isempty(config.track)
    config.track = 'autocross';
end
if ~isfield(config, 'driver_torque_max') || isempty(config.driver_torque_max)
    config.driver_torque_max = 40.0;
end
if ~isfield(config, 'driver_torque_rate_max') || isempty(config.driver_torque_rate_max)
    config.driver_torque_rate_max = 0.0;
end
if ~isfield(config, 'rl_steer_torque_rate_limit') || isempty(config.rl_steer_torque_rate_limit)
    config.rl_steer_torque_rate_limit = -1.0;
end
if ~isfield(config, 'max_episodes') || isempty(config.max_episodes)
    config.max_episodes = 600;
end
if ~isfield(config, 'max_episode_steps') || isempty(config.max_episode_steps)
    config.max_episode_steps = 8000;
end
if ~isfield(config, 'score_window') || isempty(config.score_window)
    config.score_window = 50;
end
if ~isfield(config, 'stop_training_value') || isempty(config.stop_training_value)
    config.stop_training_value = 15000;
end
if ~isfield(config, 'save_agent_frequency') || isempty(config.save_agent_frequency)
    config.save_agent_frequency = 5;
end
if ~isfield(config, 'plots') || isempty(config.plots)
    config.plots = 'none';
end
if ~isfield(config, 'verbose') || isempty(config.verbose)
    config.verbose = true;
end
if ~isfield(config, 'start_server') || isempty(config.start_server)
    config.start_server = true;
end
if ~isfield(config, 'reset_output') || isempty(config.reset_output)
    config.reset_output = false;
end
if ~isfield(config, 'export_final') || isempty(config.export_final)
    config.export_final = true;
end
if ~isfield(config, 'benchmark_after_training') || isempty(config.benchmark_after_training)
    config.benchmark_after_training = false;
end
if ~isfield(config, 'benchmark_max_checkpoints') || isempty(config.benchmark_max_checkpoints)
    config.benchmark_max_checkpoints = 5;
end
if ~isfield(config, 'benchmark_sim_time') || isempty(config.benchmark_sim_time)
    config.benchmark_sim_time = 180.0;
end
if ~isfield(config, 'rethrow_on_error') || isempty(config.rethrow_on_error)
    config.rethrow_on_error = true;
end
if ~isfield(config, 'resume_agent_file') || isempty(config.resume_agent_file)
    config.resume_agent_file = '';
else
    config.resume_agent_file = resolve_project_path(config.project_root, config.resume_agent_file);
end
if ~isfield(config, 'env_overrides') || isempty(config.env_overrides)
    config.env_overrides = struct();
end
config.run_manifest_path = fullfile(config.output_dir, 'run_manifest.json');
config.crash_manifest_path = fullfile(config.output_dir, 'crash_manifest.json');
end

function result = initial_result(config)
result = struct();
result.schema_version = 'autocross_fine_tune/v2';
result.status = 'initialized';
result.generated_by = 'fine_tune_autocross';
result.generated_at_utc = utc_now();
result.run_id = config.run_id;
result.project_root = config.project_root;
result.output_dir = config.output_dir;
result.track = config.track;
result.max_episodes = config.max_episodes;
result.max_episode_steps = config.max_episode_steps;
result.save_agent_frequency = config.save_agent_frequency;
result.driver_torque_max = config.driver_torque_max;
result.driver_torque_rate_max = config.driver_torque_rate_max;
result.rl_steer_torque_rate_limit = config.rl_steer_torque_rate_limit;
result.source_agent_file = '';
result.source_agent_variable = '';
result.resume_episode = 0;
result.latest_checkpoint = '';
end

function [agent_file, resume_episode] = resolve_start_agent(config)
if ~isempty(config.resume_agent_file)
    agent_file = config.resume_agent_file;
    resume_episode = infer_episode(agent_file);
    return;
end

[agent_file, resume_episode] = findLatestAgentByEpisode(config.output_dir);
if ~isempty(agent_file)
    return;
end

search_paths = {
    fullfile(config.project_root, 'rl_results', 'autocross_winged_flexible_ft_v6_40Nm')
    fullfile(config.project_root, 'rl_results', 'autocross_winged_flexible_ft_v5')
    fullfile(config.project_root, 'rl_results', 'autocross_winged_flexible_ft_v4')
    };

for i = 1:numel(search_paths)
    if i == 3
        preferred = fullfile(search_paths{i}, 'Agent250.mat');
        if isfile(preferred)
            agent_file = preferred;
            resume_episode = 0;
            return;
        end
    end
    [agent_file, ~] = findLatestAgentByEpisode(search_paths{i});
    if ~isempty(agent_file)
        resume_episode = 0;
        return;
    end
end

agent_file = fullfile(config.project_root, 'rl_results', ...
    'autocross_winged_flexible_ft_v3', 'Agent89.mat');
resume_episode = 0;
if ~isfile(agent_file)
    error('fine_tune_autocross:MissingSourceAgent', ...
        'No source agent checkpoint was found.');
end
end

function configure_environment(env, config, resume_episode)
env.MaxEpisodeSteps = config.max_episode_steps;
env.EpisodeIndexOffset = max(0, resume_episode);

env.W_progress = 12.0;
env.W_speed = 1.5;
env.W_boundary = 50.0;
env.W_centering = 4.0;
env.W_lap = 400.0;
env.W_heading = 3.0;
env.W_alive = -8.0;
env.W_slip = 0.5;
env.W_spin = 15.0;
env.W_jerk = 0.06;
env.W_steerReversal = 1.2;
env.W_twist = 0.25;
env.W_steerRate = 0.04;
env.W_steerCommand = 0.03;
env.W_steerSaturation = 0.35;
env.OffTrackMargin = -0.10;
env.OffTrackGraceSteps = ceil(0.25 / env.ControlDt);

env.CrawlWindowSteps = 100;
env.CrawlMinProgress = 5.0;
env.W_lowSpeedBrake = 14.0;
env.LowSpeedBrakeThreshold = 3.0;
env.W_previewSteer = 20.0;
env.PreviewSteerSpeedThreshold = 14.0;
env.PreviewCurvatureThreshold = 0.010;
env.RollingStartSpeed = 0.0;
env.RollingStartThrottle = 0.0;
env.RollingStartMaxSteps = 0;
env.RollingStartHoldEpisodes = 0;
env.RollingStartFadeEpisodes = 0;

applyRLEnvOverrides(env, config.env_overrides, 'fine_tune_autocross override');
end

function [agentFile, bestEpisode] = findLatestAgentByEpisode(resultDir)
agentFile = '';
bestEpisode = 0;
if ~exist(resultDir, 'dir')
    return;
end

files = dir(fullfile(resultDir, 'Agent*.mat'));
bestDatenum = -Inf;
for k = 1:numel(files)
    episode = infer_episode(files(k).name);
    if episode > bestEpisode || (episode == bestEpisode && files(k).datenum > bestDatenum)
        bestEpisode = episode;
        bestDatenum = files(k).datenum;
        agentFile = fullfile(resultDir, files(k).name);
    end
end
end

function episode = infer_episode(pathIn)
[~, name] = fileparts(pathIn);
tokens = regexp(name, '^Agent(\d+)$', 'tokens', 'once');
if isempty(tokens)
    episode = 0;
else
    episode = str2double(tokens{1});
end
end

function cleanup_environment(env)
if ~isempty(env) && isvalid(env)
    delete(env);
end
end

function ensure_dir(pathIn)
if ~exist(pathIn, 'dir')
    mkdir(pathIn);
end
end

function write_json(pathIn, data)
parent = fileparts(pathIn);
if ~isempty(parent) && ~exist(parent, 'dir')
    mkdir(parent);
end
fid = fopen(pathIn, 'w');
if fid == -1
    error('fine_tune_autocross:WriteFailed', 'Failed to open %s.', pathIn);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', jsonencode(data, 'PrettyPrint', true));
end

function pathOut = resolve_project_path(projectRoot, pathIn)
pathIn = char(string(pathIn));
if startsWith(pathIn, filesep) || ~isempty(regexp(pathIn, '^[A-Za-z]:[\\/]', 'once'))
    pathOut = pathIn;
else
    pathOut = fullfile(projectRoot, pathIn);
end
end

function out = utc_now()
out = char(datetime('now', 'TimeZone', 'UTC', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z'''));
end
