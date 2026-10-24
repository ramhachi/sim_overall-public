% trainCircleV3.m — 2D行動空間での円形コース学習 + エクスポート
% 使い方: cd matlab/rl_driver && trainCircleV3

%% 環境作成（2D行動空間を強制確認）
env = FSAEEnvironment('circle');
actInfo = getActionInfo(env);
obsInfo = getObservationInfo(env);
fprintf('Action dim: %d\n', actInfo.Dimension(1));
assert(actInfo.Dimension(1) == 2, 'Expected 2D action space');

%% SAC エージェント作成
agent = createSACAgent(env);
fprintf('Agent created (2D action)\n');

%% 学習オプション
outDir = fullfile(pwd, '../../rl_results/circle_v3');
if ~exist(outDir, 'dir'), mkdir(outDir); end

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 300, ...
    'MaxStepsPerEpisode', env.MaxEpisodeSteps, ...
    'ScoreAveragingWindowLength', 20, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 500, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 100, ...
    'SaveAgentDirectory', outDir);

%% 学習
fprintf('=== Starting 2D action training (circle) ===\n');
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
