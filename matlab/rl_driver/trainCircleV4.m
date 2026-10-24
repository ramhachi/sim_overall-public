% trainCircleV4.m — 両方向円形コース学習（CW+CCW）
% 使い方: cd matlab/rl_driver && trainCircleV4

%% 環境作成（2D行動空間、CW/CCWランダム選択）
env = FSAEEnvironment('circle');
env.TrackPool = {'circle', 'circle_cw'};  % 各エピソードでランダム選択
actInfo = getActionInfo(env);
obsInfo = getObservationInfo(env);
fprintf('Action dim: %d\n', actInfo.Dimension(1));
assert(actInfo.Dimension(1) == 2, 'Expected 2D action space');

%% SAC エージェント作成
agent = createSACAgent(env);
fprintf('Agent created (2D action, bidirectional)\n');

%% 学習オプション
outDir = fullfile(pwd, '../../rl_results/circle_v4');
if ~exist(outDir, 'dir'), mkdir(outDir); end

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 500, ...
    'MaxStepsPerEpisode', env.MaxEpisodeSteps, ...
    'ScoreAveragingWindowLength', 30, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 600, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 100, ...
    'SaveAgentDirectory', outDir);

%% 学習
fprintf('=== Starting bidirectional circle training (CW+CCW) ===\n');
trainingStats = train(agent, env, trainOpts);

%% 保存
save(fullfile(outDir, 'trained_agent.mat'), 'agent');
save(fullfile(outDir, 'training_stats.mat'), 'trainingStats');
fprintf('Training complete.\n');

%% C++エクスポート
exportActorForCpp(agent, fullfile(pwd, 'rl_actor_export'));
exportActorForCpp(agent, outDir);
fprintf('Export complete.\n');

%% クリーンアップ
delete(env);
fprintf('Done.\n');
