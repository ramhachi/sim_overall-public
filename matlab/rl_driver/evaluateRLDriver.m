function evaluateRLDriver(agentOrPath, trackName, vehicleParams)
    % EVALUATERLDRIVER 学習済みRLドライバーの評価と可視化
    %
    % evaluateRLDriver('rl_results/circle/trained_agent.mat', 'circle')
    % evaluateRLDriver(agent, 'skidpad')
    %
    % 出力:
    %   - 軌跡プロット（x-y, トラック境界付き）
    %   - 速度プロファイル vs 曲率
    %   - g-gダイアグラム（摩擦円使用率）
    %   - ステアリング/スロットル/ブレーキ時系列
    %   - PIDドライバーとの比較表
    ensure_rl_driver_paths();

    if nargin < 2, trackName = 'circle'; end
    if nargin < 3, vehicleParams = struct(); end
    
    %% エージェント読み込み
    if ischar(agentOrPath) || isstring(agentOrPath)
        agent = loadRLAgentFromFile(char(agentOrPath));
    else
        agent = agentOrPath;
    end
    assertAgentObservationDimension(agent, [], 'Evaluation agent');
    actorModel = getDeterministicActorModel(agent);
    
    %% 環境作成
    env = FSAEEnvironment(trackName, vehicleParams);
    envCleanup = onCleanup(@() cleanupEnv(env));
    
    %% エピソード実行（100ステップごとにログ）
    obs = reset(env);
    
    maxSteps = env.MaxEpisodeSteps;
    log.obs = zeros(numel(obs), maxSteps);
    log.action = zeros(2, maxSteps);
    log.reward = zeros(1, maxSteps);
    log.vx = zeros(1, maxSteps);
    log.yawRate = zeros(1, maxSteps);
    log.lateralDev = zeros(1, maxSteps);
    log.lapProgress = zeros(1, maxSteps);
    
    totalReward = 0;
    nSteps = 0;
    
    for t = 1:maxSteps
        % 決定論的行動（学習後は平均値のみ使用）
        action = predictDeterministicAction(actorModel, obs);
        
        [obs, reward, isDone, loggedSignals] = step(env, action);
        
        log.obs(:, t) = obs;
        log.action(:, t) = action;
        log.reward(t) = reward;
        log.vx(t) = loggedSignals.vx;
        log.yawRate(t) = loggedSignals.yawRate;
        log.lateralDev(t) = loggedSignals.lateralDev;
        log.lapProgress(t) = loggedSignals.lapProgress;
        
        totalReward = totalReward + reward;
        nSteps = t;
        
        if isDone
            break;
        end
    end
    
    % ログをトリム
    log.obs = log.obs(:, 1:nSteps);
    log.action = log.action(:, 1:nSteps);
    log.reward = log.reward(1:nSteps);
    log.vx = log.vx(1:nSteps);
    log.yawRate = log.yawRate(1:nSteps);
    log.lateralDev = log.lateralDev(1:nSteps);
    log.lapProgress = log.lapProgress(1:nSteps);
    
    %% 結果サマリー
    lapTime = nSteps * env.ControlDt;
    fprintf('\n=== RL Driver Evaluation ===\n');
    fprintf('Track: %s\n', trackName);
    fprintf('Steps: %d (%.1f s)\n', nSteps, lapTime);
    fprintf('Total Reward: %.1f\n', totalReward);
    fprintf('Avg Reward/Step: %.3f\n', totalReward / nSteps);
    fprintf('Lap completed: %d\n', env.LapCompleted);
    fprintf('Termination reason: %s\n', env.LastTerminationReason);
    
    %% 可視化
    figure('Position', [100 100 1400 900], 'Name', 'RL Driver Evaluation');
    
    time = (0:nSteps-1) * env.ControlDt;
    
    % 1. 操作入力時系列
    subplot(2, 3, 1);
    plot(time, log.action(1, :), 'b-', 'LineWidth', 1.2); hold on;
    % accel → throttle/brake に分解して表示
    accel = log.action(2, :);
    throttle_vis = max(0, accel);
    brake_vis = max(0, -accel);
    plot(time, throttle_vis, 'g-', 'LineWidth', 1.2);
    plot(time, brake_vis, 'r-', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Action [-]');
    legend('Steer', 'Throttle', 'Brake', 'Location', 'best');
    title('Control Inputs');
    grid on;
    
    % 2. 速度（観測から復元: obs[1] = vx正規化）
    subplot(2, 3, 2);
    plot(time, log.vx * 3.6, 'b-', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Speed [km/h]');
    title('Vehicle Speed');
    grid on;
    
    % 3. 横偏差
    subplot(2, 3, 3);
    plot(time, log.lateralDev, 'r-', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Lateral Deviation [m]');
    title('Lateral Deviation');
    grid on;
    yline(0, 'k--');
    
    % 4. 報酬
    subplot(2, 3, 4);
    plot(time, log.reward, 'b-', 'LineWidth', 0.8); hold on;
    plot(time, cumsum(log.reward) / (1:nSteps), 'r-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Reward');
    legend('Instant', 'Running Avg', 'Location', 'best');
    title('Reward');
    grid on;
    
    % 5. ラップ進行率
    subplot(2, 3, 5);
    plot(time, log.lapProgress * 100, 'b-', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Progress [%]');
    title('Lap Progress');
    grid on;
    
    % 6. ヨーレート vs ステアリング
    subplot(2, 3, 6);
    scatter(log.action(1, :) * 0.45 * 180/pi, log.yawRate * 180/pi, 2, time, 'filled');
    xlabel('Steering [deg]'); ylabel('Yaw Rate [deg/s]');
    title('Steering vs Yaw Rate');
    colorbar; colormap(jet);
    grid on;
    
    sgtitle(sprintf('RL Driver: %s — %.1fs, Reward=%.0f', trackName, lapTime, totalReward));
    
    %% 結果保存
    saveDir = fullfile('rl_results', trackName);
    if ~exist(saveDir, 'dir'), mkdir(saveDir); end
    saveas(gcf, fullfile(saveDir, 'evaluation.png'));
    fprintf('Evaluation saved to: %s/evaluation.png\n', saveDir);
end

function cleanupEnv(env)
    if ~isempty(env) && isvalid(env)
        delete(env);
    end
end
