% trainV10_finetune.m — v10 Phase2エージェントの精度ファインチューニング
% 目的: 横方向誤差を1.5m以下に抑えつつ速度を維持
% v10p2ベース: circle 11.2s/1.81m, skidpad 21.3s/1.38m, autocross 86.5s/2.33m

fprintf('=== v10 Precision Fine-tuning (400 eps) ===\n');

% v10 Phase2エージェントをロード
loaded = load('../../rl_results/v10_phase2/trained_agent.mat');
if isfield(loaded, 'agent')
    agent = loaded.agent;
else
    fn = fieldnames(loaded);
    agent = loaded.(fn{1});
end
fprintf('Loaded v10p2 agent.\n');

% 環境セットアップ（多様なトラック、精度重視）
env = FSAEEnvironment('circle');
env.TrackPool = {'circle', 'circle_cw', ...
    'circle_R15', 'circle_cw_R15', ...
    'skidpad', 'skidpad', ...
    'autocross', 'autocross', 'autocross'};
env.MaxEpisodeSteps = 8000;

% 精度最大化の報酬重み
env.W_progress = 10.0;      % 進行報酬維持
env.W_speed = 0.8;           % 速度ボーナス適度
env.W_boundary = 80.0;       % 境界ペナルティ最大化（コース逸脱をゼロに）
env.W_centering = 8.0;       % センタリング最大化
env.W_lap = 300.0;
env.W_heading = 4.0;         % ヘディング精度最大
env.W_alive = 0.1;
env.W_slip = 0.5;
env.W_spin = 15.0;
env.W_jerk = 0.05;

outDir = fullfile(pwd, '../../rl_results/v10_finetune');
if ~exist(outDir, 'dir'), mkdir(outDir); end

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 400, ...
    'MaxStepsPerEpisode', env.MaxEpisodeSteps, ...
    'ScoreAveragingWindowLength', 30, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1e9, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 200, ...
    'SaveAgentDirectory', outDir);

stats = train(agent, env, trainOpts);
save(fullfile(outDir, 'trained_agent.mat'), 'agent');
save(fullfile(outDir, 'training_stats.mat'), 'stats');
R = stats.EpisodeReward;
fprintf('Fine-tune: %d eps, max=%.0f, last30avg=%.0f\n', ...
    length(R), max(R), mean(R(end-min(29,length(R)-1):end)));

%% エクスポート
exportActorForCpp(agent, fullfile(pwd, 'rl_actor_export'));
exportActorForCpp(agent, fullfile(pwd, '../../cpp_export'));
fprintf('Export complete.\n');

delete(env);
fprintf('Fine-tuning done.\n');
