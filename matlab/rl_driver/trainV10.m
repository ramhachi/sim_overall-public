% trainV10.m — 正確なコース幅(3m)での学習 + 精度重視
% v9の問題: RL server幅5m vs test幅3m のミスマッチ
% v10: サーバー幅3mに統一、精度とスピードのバランス最適化
%
% Phase 1: 基礎操作習得（circle/skidpad, 500ep）— ゼロから
% Phase 2: オートクロス集中（1000ep）— Phase1から継続
% Phase 3: スピード最適化（500ep）— Phase2から継続

%% Phase 1: 基礎操作（500ep, 3m幅）
fprintf('=== Phase 1: Basic driving (500 eps, 3m width) ===\n');
env = FSAEEnvironment('circle');
env.TrackPool = {'circle', 'circle_cw', ...
    'circle_R20', 'circle_cw_R20', ...
    'circle_R15', 'circle_cw_R15', ...
    'circle_R10', 'circle_cw_R10'};
env.MaxEpisodeSteps = 3000;

% 精度重視の報酬（3m幅に合わせて境界ペナルティ強化）
env.W_progress = 8.0;
env.W_speed = 0.5;
env.W_boundary = 50.0;     % 大幅強化（3m幅で端に近づくペナルティ）
env.W_centering = 5.0;     % 大幅強化（中央維持を強制）
env.W_lap = 200.0;
env.W_heading = 3.0;
env.W_alive = 0.2;
env.W_slip = 0.5;
env.W_spin = 15.0;
env.W_jerk = 0.05;

agent = createSACAgent(env);

outDir1 = fullfile(pwd, '../../rl_results/v10_phase1');
if ~exist(outDir1, 'dir'), mkdir(outDir1); end

trainOpts1 = rlTrainingOptions( ...
    'MaxEpisodes', 500, ...
    'MaxStepsPerEpisode', env.MaxEpisodeSteps, ...
    'ScoreAveragingWindowLength', 30, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1e9, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 100, ...
    'SaveAgentDirectory', outDir1);

stats1 = train(agent, env, trainOpts1);
save(fullfile(outDir1, 'trained_agent.mat'), 'agent');
save(fullfile(outDir1, 'training_stats.mat'), 'stats1');
R1 = stats1.EpisodeReward;
fprintf('Phase 1: %d eps, max=%.0f, last30avg=%.0f\n', ...
    length(R1), max(R1), mean(R1(end-min(29,length(R1)-1):end)));
delete(env);

%% Phase 2: オートクロス集中（1000ep）
fprintf('=== Phase 2: Autocross focused (1000 eps) ===\n');
env2 = FSAEEnvironment('autocross');
env2.TrackPool = {'autocross', 'autocross', 'autocross', ...
    'circle', 'circle_cw', ...
    'skidpad', ...
    'circle_R15', 'circle_cw_R15'};
env2.MaxEpisodeSteps = 8000;

% 精度+速度バランス
env2.W_progress = 10.0;
env2.W_speed = 1.0;
env2.W_boundary = 50.0;
env2.W_centering = 4.0;
env2.W_lap = 300.0;
env2.W_heading = 3.0;
env2.W_alive = 0.1;
env2.W_slip = 0.5;
env2.W_spin = 15.0;
env2.W_jerk = 0.05;

outDir2 = fullfile(pwd, '../../rl_results/v10_phase2');
if ~exist(outDir2, 'dir'), mkdir(outDir2); end

trainOpts2 = rlTrainingOptions( ...
    'MaxEpisodes', 1000, ...
    'MaxStepsPerEpisode', env2.MaxEpisodeSteps, ...
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1e9, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 200, ...
    'SaveAgentDirectory', outDir2);

stats2 = train(agent, env2, trainOpts2);
save(fullfile(outDir2, 'trained_agent.mat'), 'agent');
save(fullfile(outDir2, 'training_stats.mat'), 'stats2');
R2 = stats2.EpisodeReward;
fprintf('Phase 2: %d eps, max=%.0f, last50avg=%.0f\n', ...
    length(R2), max(R2), mean(R2(end-min(49,length(R2)-1):end)));
delete(env2);

%% Phase 3: スピード最適化（500ep）— 精度を維持しつつ速度向上
fprintf('=== Phase 3: Speed optimization (500 eps) ===\n');
env3 = FSAEEnvironment('autocross');
env3.TrackPool = {'autocross', 'autocross', 'autocross', ...
    'circle', 'circle_cw', 'skidpad'};
env3.MaxEpisodeSteps = 8000;

% 速度重視（ただし境界ペナルティは維持）
env3.W_progress = 12.0;
env3.W_speed = 1.5;
env3.W_boundary = 45.0;
env3.W_centering = 3.0;
env3.W_lap = 400.0;
env3.W_heading = 2.5;
env3.W_alive = 0.05;
env3.W_slip = 0.5;
env3.W_spin = 15.0;
env3.W_jerk = 0.03;

outDir3 = fullfile(pwd, '../../rl_results/v10_phase3');
if ~exist(outDir3, 'dir'), mkdir(outDir3); end

trainOpts3 = rlTrainingOptions( ...
    'MaxEpisodes', 500, ...
    'MaxStepsPerEpisode', env3.MaxEpisodeSteps, ...
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1e9, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 200, ...
    'SaveAgentDirectory', outDir3);

stats3 = train(agent, env3, trainOpts3);
save(fullfile(outDir3, 'trained_agent.mat'), 'agent');
save(fullfile(outDir3, 'training_stats.mat'), 'stats3');
R3 = stats3.EpisodeReward;
fprintf('Phase 3: %d eps, max=%.0f, last50avg=%.0f\n', ...
    length(R3), max(R3), mean(R3(end-min(49,length(R3)-1):end)));

%% エクスポート
exportActorForCpp(agent, fullfile(pwd, 'rl_actor_export'));
exportActorForCpp(agent, fullfile(pwd, '../../cpp_export'));
fprintf('Export complete.\n');

delete(env3);
fprintf('All v10 training done. Total: %d + %d + %d = %d episodes.\n', ...
    length(R1), length(R2), length(R3), length(R1)+length(R2)+length(R3));
