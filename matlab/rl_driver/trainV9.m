% trainV9.m — 高性能最適化学習: 長時間訓練 + 速度最適化
% v8からの改善点:
%   1. 早期停止なし（全エピソード完走）
%   2. v8 Phase3エージェントから継続学習（ベースライン確保）
%   3. 速度ボーナス強化（速く走る動機を大幅増加）
%   4. 境界ペナルティ微調整（精度向上 + 攻めの運転を両立）
%   5. 2000エピソード以上の徹底学習

%% Phase 1: オートクロス集中訓練（800ep）— v8 Phase3から継続
fprintf('=== Phase 1: Autocross intensive (800 eps from v8p3) ===\n');
env = FSAEEnvironment('autocross');
env.TrackPool = {'autocross', 'autocross', 'autocross', 'autocross', ...
    'circle', 'circle_cw', 'skidpad'};
env.MaxEpisodeSteps = 8000;

% 速度重視の報酬重み
env.W_progress = 10.0;    % 進行報酬増加
env.W_speed = 1.5;        % 速度ボーナス大幅増加
env.W_boundary = 35.0;    % 境界ペナルティ維持（精度確保）
env.W_centering = 1.5;    % センタリング若干下げ（攻めのラインを許容）
env.W_lap = 300.0;        % ラップ完了ボーナス大幅増加
env.W_heading = 2.5;      % ヘディング精度向上
env.W_alive = 0.1;        % 生存ボーナス下げ（停滞抑制）

% v8 Phase3エージェントをロード
fprintf('Loading v8 phase3 agent...\n');
loaded = load('../../rl_results/v8_phase3/trained_agent.mat');
if isfield(loaded, 'agent')
    agent = loaded.agent;
elseif isfield(loaded, 'saved_agent')
    agent = loaded.saved_agent;
else
    fn = fieldnames(loaded);
    agent = loaded.(fn{1});
end
fprintf('Agent loaded successfully.\n');

outDir1 = fullfile(pwd, '../../rl_results/v9_phase1');
if ~exist(outDir1, 'dir'), mkdir(outDir1); end

trainOpts1 = rlTrainingOptions( ...
    'MaxEpisodes', 800, ...
    'MaxStepsPerEpisode', env.MaxEpisodeSteps, ...
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1e9, ...  % 実質的に停止しない
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 500, ...
    'SaveAgentDirectory', outDir1);

stats1 = train(agent, env, trainOpts1);
save(fullfile(outDir1, 'trained_agent.mat'), 'agent');
save(fullfile(outDir1, 'training_stats.mat'), 'stats1');
R1 = stats1.EpisodeReward;
fprintf('Phase 1: %d eps, max=%.0f, last50avg=%.0f\n', ...
    length(R1), max(R1), mean(R1(end-min(49,length(R1)-1):end)));
delete(env);

%% Phase 2: 精度強化（600ep）— 横方向誤差を減らす
fprintf('=== Phase 2: Precision tuning (600 eps) ===\n');
env2 = FSAEEnvironment('autocross');
env2.TrackPool = {'autocross', 'autocross', ...
    'circle', 'circle_cw', ...
    'circle_R15', 'circle_cw_R15', ...
    'skidpad'};
env2.MaxEpisodeSteps = 8000;

% 精度重視の報酬重み
env2.W_progress = 10.0;
env2.W_speed = 1.0;       % 速度は維持
env2.W_boundary = 50.0;   % 境界ペナルティ大幅強化
env2.W_centering = 4.0;   % センタリング大幅強化
env2.W_lap = 300.0;
env2.W_heading = 3.0;     % ヘディング精度更に向上
env2.W_alive = 0.1;

outDir2 = fullfile(pwd, '../../rl_results/v9_phase2');
if ~exist(outDir2, 'dir'), mkdir(outDir2); end

trainOpts2 = rlTrainingOptions( ...
    'MaxEpisodes', 600, ...
    'MaxStepsPerEpisode', env2.MaxEpisodeSteps, ...
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1e9, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 500, ...
    'SaveAgentDirectory', outDir2);

stats2 = train(agent, env2, trainOpts2);
save(fullfile(outDir2, 'trained_agent.mat'), 'agent');
save(fullfile(outDir2, 'training_stats.mat'), 'stats2');
R2 = stats2.EpisodeReward;
fprintf('Phase 2: %d eps, max=%.0f, last50avg=%.0f\n', ...
    length(R2), max(R2), mean(R2(end-min(49,length(R2)-1):end)));
delete(env2);

%% Phase 3: 速度最適化（800ep）— ラップタイム短縮
fprintf('=== Phase 3: Speed optimization (800 eps) ===\n');
env3 = FSAEEnvironment('autocross');
env3.TrackPool = {'autocross', 'autocross', 'autocross', ...
    'circle', 'circle_cw', 'skidpad'};
env3.MaxEpisodeSteps = 8000;

% 速度最大化の報酬重み
env3.W_progress = 12.0;   % 進行報酬最大
env3.W_speed = 2.0;       % 速度ボーナス最大
env3.W_boundary = 40.0;   % 境界ペナルティはphase2より緩め
env3.W_centering = 2.0;   % センタリングは緩め（速度重視）
env3.W_lap = 500.0;       % ラップ完了ボーナス最大
env3.W_heading = 2.0;
env3.W_alive = 0.05;

outDir3 = fullfile(pwd, '../../rl_results/v9_phase3');
if ~exist(outDir3, 'dir'), mkdir(outDir3); end

trainOpts3 = rlTrainingOptions( ...
    'MaxEpisodes', 800, ...
    'MaxStepsPerEpisode', env3.MaxEpisodeSteps, ...
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1e9, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 500, ...
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
fprintf('All v9 training done. Total: %d + %d + %d = %d episodes.\n', ...
    length(R1), length(R2), length(R3), length(R1)+length(R2)+length(R3));
