% trainAutocrossV6.m — オートクロスコース学習（Phase2エージェントから継続学習）
% 使い方: cd matlab/rl_driver && trainAutocrossV6

%% Phase 2エージェントをロード
fprintf('Loading Phase 2 agent...\n');
s = load(fullfile(pwd, '../../rl_results/multi_v5_phase2/trained_agent.mat'));
agent = s.agent;
fprintf('Agent loaded.\n');

%% 環境作成（混合トラック: autocross中心 + circle復習）
env = FSAEEnvironment('autocross');
env.TrackPool = {'autocross', 'autocross', 'autocross', 'circle', 'circle_cw', 'skidpad'};
env.MaxEpisodeSteps = 8000;  % 160秒（autocross完走に十分）

actInfo = getActionInfo(env);
fprintf('Action dim: %d, MaxSteps: %d\n', actInfo.Dimension(1), env.MaxEpisodeSteps);

%% 学習オプション
outDir = fullfile(pwd, '../../rl_results/autocross_v6');
if ~exist(outDir, 'dir'), mkdir(outDir); end

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 500, ...
    'MaxStepsPerEpisode', env.MaxEpisodeSteps, ...
    'ScoreAveragingWindowLength', 30, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 800, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 100, ...
    'SaveAgentDirectory', outDir);

%% 学習
fprintf('=== Starting autocross training (from Phase 2) ===\n');
trainingStats = train(agent, env, trainOpts);

%% 保存
save(fullfile(outDir, 'trained_agent.mat'), 'agent');
save(fullfile(outDir, 'training_stats.mat'), 'trainingStats');
fprintf('Training complete.\n');

%% C++エクスポート
exportActorForCpp(agent, fullfile(pwd, 'rl_actor_export'));
exportActorForCpp(agent, outDir);
fprintf('Export complete.\n');

delete(env);
fprintf('Done.\n');
