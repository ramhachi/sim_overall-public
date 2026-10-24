function exportActorForCpp(agent, outputDir, vehicleParams)
    % EXPORTACTORFORCPP 学習済みActorネットワークをC++用テキストにエクスポート
    %
    % exportActorForCpp(agent, 'rl_results/circle')
    % exportActorForCpp(agent, 'rl_results/circle', vehicleParams)
    %
    % 出力ファイル:
    %   rl_actor_weights.txt       - Mean-pathのFC重み・バイアス（fcStdは除外）
    %   rl_normalization_params.txt - 観測正規化パラメータ
    %   rl_actor_test_data.csv     - C++一致性検証用テストデータ
    ensure_rl_driver_paths();

    if nargin < 2, outputDir = 'rl_results'; end
    if nargin < 3, vehicleParams = struct(); end
    if ~exist(outputDir, 'dir'), mkdir(outputDir); end
    assertAgentObservationDimension(agent, [], 'Export agent');

    %% Actorネットワーク取得
    actor = getActor(agent);
    try
        actorNet = getModel(actor);
    catch
        actorNet = actor.Model;
    end

    %% 行動空間の次元を取得
    actInfo = getActionInfo(agent);
    actDim = actInfo.Dimension(1);
    fprintf('Action dimension: %d\n', actDim);
    
    %% LayerNormをFC層に関連付け（mean-pathのみ抽出）
    foldedLayers = extractMeanPathLayers(actorNet);
    
    %% 重みファイル出力（mean-pathのみ）
    fid = fopen(fullfile(outputDir, 'rl_actor_weights.txt'), 'w');
    for i = 1:length(foldedLayers)
        layer = foldedLayers{i};
        fprintf(fid, 'LAYER %s\n', layer.name);
        fprintf(fid, 'WEIGHTS %d %d\n', size(layer.W, 1), size(layer.W, 2));
        for r = 1:size(layer.W, 1)
            for c = 1:size(layer.W, 2)
                fprintf(fid, '%.15e\n', layer.W(r, c));
            end
        end
        fprintf(fid, 'BIAS %d\n', length(layer.b));
        for k = 1:length(layer.b)
            fprintf(fid, '%.15e\n', layer.b(k));
        end
        if ~isempty(layer.ln_gamma)
            fprintf(fid, 'LN_GAMMA %d\n', length(layer.ln_gamma));
            for k = 1:length(layer.ln_gamma)
                fprintf(fid, '%.15e\n', layer.ln_gamma(k));
            end
            fprintf(fid, 'LN_BETA %d\n', length(layer.ln_beta));
            for k = 1:length(layer.ln_beta)
                fprintf(fid, '%.15e\n', layer.ln_beta(k));
            end
        end
    end
    fclose(fid);
    
    %% 正規化パラメータ出力（FSAEEnvironmentと同一の範囲）
    exportVehicleParams = resolve_rl_vehicle_params(vehicleParams);
    bounds = get_rl_observation_bounds(exportVehicleParams);
    if exportVehicleParams.steering.torque_mode
        steerScale = exportVehicleParams.steering.T_driver_max;
    else
        steerScale = exportVehicleParams.steering.max_angle;
    end
    
    fid = fopen(fullfile(outputDir, 'rl_normalization_params.txt'), 'w');
    fprintf(fid, 'OBS_BOUNDS %d\n', size(bounds, 1));
    for i = 1:size(bounds, 1)
        fprintf(fid, '%.15e %.15e\n', bounds(i, 1), bounds(i, 2));
    end
    if actDim == 2
        fprintf(fid, 'ACTION_SCALE %d\n', 2);
        fprintf(fid, '%.15e\n', steerScale);  % steering torque or angle scale
        fprintf(fid, '%.15e\n', 1.0);   % accel_max
    else
        fprintf(fid, 'ACTION_SCALE %d\n', 3);
        fprintf(fid, '%.15e\n', steerScale);  % steering torque or angle scale
        fprintf(fid, '%.15e\n', 1.0);   % throttle_max
        fprintf(fid, '%.15e\n', 1.0);   % brake_max
    end
    fclose(fid);
    
    %% テストデータ生成
    nTest = 100;
    rng(42);
    nObs = size(bounds, 1);
    testInputs = 2 * rand(nObs, nTest) - 1;  % [-1, 1] ランダム入力
    testOutputs = zeros(actDim, nTest);

    for i = 1:nTest
        obs = dlarray(testInputs(:, i), 'CB');
        result = predict(actorNet, obs);
        if iscell(result)
            actMean = result{1};
        else
            actMean = result;
        end
        vals = safeExtract(actMean);
        testOutputs(:, i) = vals(1:actDim);
    end
    
    % CSV出力
    nCols = nObs + actDim;
    data = [testInputs; testOutputs]';
    headers = cell(1, nCols);
    for i = 1:nObs, headers{i} = sprintf('obs_%d', i); end
    if actDim == 2
        headers{nObs + 1} = 'act_steer'; headers{nObs + 2} = 'act_accel';
    else
        headers{nObs + 1} = 'act_steer';
        headers{nObs + 2} = 'act_throttle';
        headers{nObs + 3} = 'act_brake';
    end
    
    fid = fopen(fullfile(outputDir, 'rl_actor_test_data.csv'), 'w');
    fprintf(fid, '%s\n', strjoin(headers, ','));
    for i = 1:nTest
        fprintf(fid, '%.15e', data(i, 1));
        for j = 2:nCols
            fprintf(fid, ',%.15e', data(i, j));
        end
        fprintf(fid, '\n');
    end
    fclose(fid);
    
    fprintf('Export complete: %s\n', outputDir);
    fprintf('  rl_actor_weights.txt (%d mean-path layers)\n', length(foldedLayers));
    fprintf('  rl_normalization_params.txt (act_dim=%d)\n', actDim);
    fprintf('  rl_actor_test_data.csv (%d test points)\n', nTest);
end

function layers = extractMeanPathLayers(net)
    % SAC Actorから mean-path のFC層のみ抽出（fcStdは除外）
    % 構造: fc1→LN→ReLU→fc2→LN→ReLU→fc3→LN→ReLU→{fcMean, fcStd}
    
    layers = {};
    allLayers = net.Layers;
    
    i = 1;
    while i <= length(allLayers)
        layerClass = class(allLayers(i));
        
        if contains(layerClass, 'FullyConnected')
            layerName = allLayers(i).Name;
            
            % fcStd（標準偏差パス）はスキップ
            if contains(layerName, 'Std', 'IgnoreCase', true)
                i = i + 1;
                continue;
            end
            
            W = safeExtract(allLayers(i).Weights);
            b = safeExtract(allLayers(i).Bias);
            
            % 次の層がLayerNormかチェック
            if i + 1 <= length(allLayers) && contains(class(allLayers(i+1)), 'LayerNormalization')
                lnLayer = allLayers(i+1);
                gamma = safeExtract(lnLayer.Scale);
                beta  = safeExtract(lnLayer.Offset);
                
                entry.name  = layerName;
                entry.W     = W;
                entry.b     = b(:);
                entry.ln_gamma = gamma(:);
                entry.ln_beta  = beta(:);
                layers{end+1} = entry; %#ok<AGROW>
                i = i + 2;
            else
                entry.name  = layerName;
                entry.W     = W;
                entry.b     = b(:);
                entry.ln_gamma = [];
                entry.ln_beta  = [];
                layers{end+1} = entry; %#ok<AGROW>
                i = i + 1;
            end
        else
            i = i + 1;
        end
    end
end

function v = safeExtract(x)
    if isa(x, 'dlarray')
        v = double(extractdata(x));
    else
        v = double(x);
    end
end
