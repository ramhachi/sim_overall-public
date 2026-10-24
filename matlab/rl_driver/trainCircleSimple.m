function trainCircleSimple(varargin)
    % TRAINCIRCLESIMPLE 円形コース完走を最優先したシンプル学習スクリプト
    %
    % trainCircleSimple()                          % デフォルト設定で学習
    % trainCircleSimple('MaxEpisodes', 50000)      % エピソード数指定
    % trainCircleSimple('ResumeAgent', agent)       % 既存エージェントから再開
    %
    % 設計方針:
    %   - カリキュラムなし: 円形コースに特化
    %   - 2D行動空間: [steer, accel] で同時踏み問題を排除
    %   - 報酬関数: 進行報酬を主、ペナルティを従
    %   - 緩い終了条件: 探索を殺さない

    %% オプション解析
    p = inputParser;
    addParameter(p, 'MaxEpisodes', 100000);
    addParameter(p, 'SaveDir', fullfile('rl_results', 'circle_v2'));
    addParameter(p, 'ResumeAgent', []);
    addParameter(p, 'StopReward', 500);  % 平均報酬500超で停止
    parse(p, varargin{:});
    opts = p.Results;

    if ~exist(opts.SaveDir, 'dir')
        mkdir(opts.SaveDir);
    end

    %% 環境構築
    fprintf('=== Circle Simple Training (2D Action) ===\n');
    fprintf('Save Directory: %s\n', opts.SaveDir);

    env = FSAEEnvironment('circle');
    env.MaxEpisodeSteps = 3000;  % 60秒

    % 円形コース用に報酬重みを微調整
    env.W_progress = 5.0;
    env.W_speed = 0.3;
    env.W_boundary = 8.0;
    env.W_centering = 1.0;
    env.W_heading = 0.5;
    env.W_alive = 0.15;
    env.W_lap = 200.0;  % 完走ボーナスを大きく
    env.W_spin = 5.0;

    fprintf('Track: circle, Length: %.1fm\n', env.TrackLength);
    fprintf('Action space: 2D [steer, accel]\n');
    fprintf('Max episodes: %d\n', opts.MaxEpisodes);

    %% エージェント構築
    if ~isempty(opts.ResumeAgent)
        agent = opts.ResumeAgent;
        assertAgentObservationDimension(agent, getObservationInfo(env).Dimension(1), 'ResumeAgent');
        fprintf('Resuming from existing agent.\n');
    else
        agent = createSACAgent(env);
    end

    %% 学習オプション
    trainOpts = rlTrainingOptions;
    trainOpts.MaxEpisodes = opts.MaxEpisodes;
    trainOpts.MaxStepsPerEpisode = env.MaxEpisodeSteps;
    trainOpts.ScoreAveragingWindowLength = 50;
    trainOpts.StopTrainingCriteria = 'AverageReward';
    trainOpts.StopTrainingValue = opts.StopReward;
    trainOpts.SaveAgentCriteria = 'EpisodeReward';
    trainOpts.SaveAgentValue = opts.StopReward * 0.5;
    trainOpts.SaveAgentDirectory = opts.SaveDir;
    trainOpts.Plots = 'training-progress';
    trainOpts.Verbose = true;

    %% 学習実行
    fprintf('\nStarting training...\n');
    fprintf('Stop criteria: Average reward > %d\n', opts.StopReward);
    fprintf('Estimated convergence: 2000-10000 episodes\n');
    fprintf('Press Ctrl+C to stop manually.\n\n');

    trainingStats = train(agent, env, trainOpts);

    %% 結果保存
    save(fullfile(opts.SaveDir, 'trained_agent.mat'), 'agent');
    save(fullfile(opts.SaveDir, 'training_stats.mat'), 'trainingStats');

    fprintf('\n=== Training Complete ===\n');
    fprintf('Results saved to: %s\n', opts.SaveDir);

    % 簡易評価
    fprintf('\nRunning quick evaluation...\n');
    try
        evaluateRLDriver(agent, 'circle');
    catch e
        fprintf('Evaluation failed: %s\n', e.message);
    end
end
