function trainRLDriver(trackName, vehicleParams, varargin)
    % TRAINRLDRIVER RLドライバーの学習を実行
    %
    % trainRLDriver('circle')
    % trainRLDriver('skidpad', struct('mass', struct('m_total', 250)))
    % trainRLDriver('circle', struct(), 'MaxEpisodes', 1000000)
    %
    % 計画書 §5.6 に基づく学習スクリプト
    
    if nargin < 1, trackName = 'circle'; end
    if nargin < 2, vehicleParams = struct(); end
    
    %% オプション解析
    p = inputParser;
    addParameter(p, 'MaxEpisodes', 1000000);  % 丸一日学習向けに大幅増量
    addParameter(p, 'StopReward', 2000);      % 報酬スケール変更に合わせて引き上げ
    addParameter(p, 'SaveDir', fullfile('rl_results', trackName));
    addParameter(p, 'UseParallel', false);
    addParameter(p, 'DomainRand', false);
    addParameter(p, 'RandInterval', 100);
    addParameter(p, 'RandStrength', 0.3);
    addParameter(p, 'ResumeAgent', []);  % 既存エージェントから再開
    addParameter(p, 'EnvOverrides', struct());
    addParameter(p, 'MaxStepsPerEpisode', []);
    addParameter(p, 'ScoreAveragingWindowLength', 100);
    addParameter(p, 'SaveAgentCriteria', 'EpisodeReward');
    addParameter(p, 'SaveAgentValue', []);
    addParameter(p, 'Plots', 'training-progress');
    addParameter(p, 'Verbose', true);
    parse(p, varargin{:});
    opts = p.Results;
    
    %% 出力ディレクトリ作成
    if ~exist(opts.SaveDir, 'dir')
        mkdir(opts.SaveDir);
    end
    
    %% サーバー起動確認
    fprintf('=== RL Driver Training ===\n');
    fprintf('Track: %s\n', trackName);
    fprintf('Max Episodes: %d\n', opts.MaxEpisodes);
    fprintf('Save Directory: %s\n', opts.SaveDir);
    
    %% 環境構築
    env = FSAEEnvironment(trackName, vehicleParams);
    if ~isempty(opts.MaxStepsPerEpisode)
        env.MaxEpisodeSteps = opts.MaxStepsPerEpisode;
    end
    applyRLEnvOverrides(env, opts.EnvOverrides, 'Env override');
    if opts.DomainRand
        env.DomainRandEnabled = true;
        env.DomainRandInterval = opts.RandInterval;
        env.DomainRandStrength = opts.RandStrength;
        env.DomainRandBounds = getVehicleParamBounds();
    end
    
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
    trainOpts.ScoreAveragingWindowLength = opts.ScoreAveragingWindowLength;
    trainOpts.StopTrainingCriteria = 'AverageReward';
    trainOpts.StopTrainingValue = opts.StopReward;
    trainOpts.SaveAgentCriteria = opts.SaveAgentCriteria;
    if isempty(opts.SaveAgentValue)
        switch opts.SaveAgentCriteria
            case 'EpisodeReward'
                trainOpts.SaveAgentValue = opts.StopReward * 0.8;
            case 'EpisodeCount'
                trainOpts.SaveAgentValue = max(1, floor(opts.MaxEpisodes / 10));
            otherwise
                error('SaveAgentValue must be specified for SaveAgentCriteria=%s.', opts.SaveAgentCriteria);
        end
    else
        trainOpts.SaveAgentValue = opts.SaveAgentValue;
    end
    trainOpts.SaveAgentDirectory = opts.SaveDir;
    
    % 可視化
    trainOpts.Plots = opts.Plots;
    trainOpts.Verbose = opts.Verbose;
    if isprop(trainOpts, 'UseParallel')
        trainOpts.UseParallel = opts.UseParallel;
    end
    
    %% 学習実行
    fprintf('\nStarting training...\n');
    trainingStats = train(agent, env, trainOpts);
    
    %% 結果保存
    save(fullfile(opts.SaveDir, 'trained_agent.mat'), 'agent');
    save(fullfile(opts.SaveDir, 'training_stats.mat'), 'trainingStats');
    
    fprintf('\n=== Training Complete ===\n');
    fprintf('Results saved to: %s\n', opts.SaveDir);
end
