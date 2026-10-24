function agent = createSACAgent(env)
    % CREATESACAGENT SACエージェントの構築
    %
    % 計画書 §4.1, §5.5 に基づくActor-Criticアーキテクチャ
    % Actor: FC(256)→LN→ReLU ×3 → 分岐(μ: Tanh, σ: Softplus)
    % Critic ×2: FC(256)→LN→ReLU ×3 → FC(1)
    
    obsInfo = getObservationInfo(env);
    actInfo = getActionInfo(env);
    
    obsDim = obsInfo.Dimension(1);  % torque/frame-aware vehicle-conditioned pipeline
    actDim = actInfo.Dimension(1);  % 2
    
    %% ======== Actor ネットワーク（ガウス方策）========
    commonPath = [
        featureInputLayer(obsDim, 'Name', 'obs')
        fullyConnectedLayer(256, 'Name', 'fc1')
        layerNormalizationLayer('Name', 'ln1')
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(256, 'Name', 'fc2')
        layerNormalizationLayer('Name', 'ln2')
        reluLayer('Name', 'relu2')
        fullyConnectedLayer(128, 'Name', 'fc3')
        layerNormalizationLayer('Name', 'ln3')
        reluLayer('Name', 'relu3')
    ];
    
    % 平均出力パス
    meanPath = [
        fullyConnectedLayer(actDim, 'Name', 'fcMean')
        tanhLayer('Name', 'tanhMean')
    ];
    
    % 標準偏差出力パス
    stdPath = [
        fullyConnectedLayer(actDim, 'Name', 'fcStd')
        softplusLayer('Name', 'softplusStd')
    ];
    
    actorNet = layerGraph(commonPath);
    actorNet = addLayers(actorNet, meanPath);
    actorNet = addLayers(actorNet, stdPath);
    actorNet = connectLayers(actorNet, 'relu3', 'fcMean');
    actorNet = connectLayers(actorNet, 'relu3', 'fcStd');
    actorNet = dlnetwork(actorNet);
    
    actor = rlContinuousGaussianActor(actorNet, obsInfo, actInfo, ...
        'ObservationInputNames', 'obs', ...
        'ActionMeanOutputNames', 'tanhMean', ...
        'ActionStandardDeviationOutputNames', 'softplusStd');
    
    %% ======== Critic ネットワーク (×2) ========
    criticNet = createCriticNetwork(obsDim, actDim);
    
    critic1 = rlQValueFunction(criticNet, obsInfo, actInfo, ...
        'ObservationInputNames', 'obs_in', ...
        'ActionInputNames', 'act_in');
    
    % 2つ目のCriticは独立した重みで初期化
    criticNet2 = createCriticNetwork(obsDim, actDim);
    critic2 = rlQValueFunction(criticNet2, obsInfo, actInfo, ...
        'ObservationInputNames', 'obs_in', ...
        'ActionInputNames', 'act_in');
    
    %% ======== SAC エージェント ========
    agentOpts = rlSACAgentOptions;
    agentOpts.SampleTime = env.ControlDt;
    agentOpts.DiscountFactor = 0.99;
    agentOpts.TargetSmoothFactor = 5e-3;
    agentOpts.ExperienceBufferLength = 1e6;
    agentOpts.MiniBatchSize = 256;
    
    % Actor 学習率
    agentOpts.ActorOptimizerOptions = rlOptimizerOptions( ...
        'LearnRate', 3e-4, ...
        'GradientThreshold', 1.0);
    
    % Critic 学習率
    agentOpts.CriticOptimizerOptions = rlOptimizerOptions( ...
        'LearnRate', 3e-4, ...
        'GradientThreshold', 1.0);
    
    % エントロピー温度の自動調整
    agentOpts.EntropyWeightOptions.LearnRate = 3e-4;
    agentOpts.EntropyWeightOptions.TargetEntropy = -actDim;  % = -actDim (2D: -2, 3D: -3)
    
    agent = rlSACAgent(actor, [critic1, critic2], agentOpts);
    
    fprintf('SAC Agent created.\n');
    fprintf('  Actor params:  ~%dk\n', round(countParams(actorNet)/1000));
    fprintf('  Critic params: ~%dk × 2\n', round(countParams(criticNet)/1000));
end

function net = createCriticNetwork(obsDim, actDim)
    % 観測と行動を結合して入力するCriticネットワーク
    obsPath = featureInputLayer(obsDim, 'Name', 'obs_in');
    actPath = featureInputLayer(actDim, 'Name', 'act_in');
    
    concatPath = concatenationLayer(1, 2, 'Name', 'concat');
    
    hiddenPath = [
        fullyConnectedLayer(256, 'Name', 'cfc1')
        layerNormalizationLayer('Name', 'cln1')
        reluLayer('Name', 'crelu1')
        fullyConnectedLayer(256, 'Name', 'cfc2')
        layerNormalizationLayer('Name', 'cln2')
        reluLayer('Name', 'crelu2')
        fullyConnectedLayer(128, 'Name', 'cfc3')
        layerNormalizationLayer('Name', 'cln3')
        reluLayer('Name', 'crelu3')
        fullyConnectedLayer(1, 'Name', 'qvalue')
    ];
    
    net = layerGraph();
    net = addLayers(net, obsPath);
    net = addLayers(net, actPath);
    net = addLayers(net, concatPath);
    net = addLayers(net, hiddenPath);
    net = connectLayers(net, 'obs_in', 'concat/in1');
    net = connectLayers(net, 'act_in', 'concat/in2');
    net = connectLayers(net, 'concat', 'cfc1');
    net = dlnetwork(net);
end

function n = countParams(net)
    n = 0;
    params = net.Learnables;
    for i = 1:size(params, 1)
        n = n + numel(params.Value{i});
    end
end
