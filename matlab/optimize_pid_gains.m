%% PIDゲイン最適化スクリプト
% ベイズ最適化を使用してPIDゲインを自動チューニング
% VehicleSimRunnerを使用して永続的なDockerコンテナで実行し、
% JSONで結果を直接取得することでCSV読み書きのオーバーヘッドを削減。
%
% 使い方:
%   cd <project_root>/matlab
%   optimize_pid_gains('figure_eight', 25, 20)
%   optimize_pid_gains('ASE_autocross', 0, 180, 5.0, 30)

function results = optimize_pid_gains(track_type, track_radius, sim_time, initial_speed, n_iter, params_file)
    % デフォルト引数
    if nargin < 1
        track_type = 'figure_eight';
    end
    if nargin < 2
        track_radius = 25.0;
    end
    if nargin < 3
        sim_time = 15.0;  % 最適化では短めに
    end
    if nargin < 4
        initial_speed = 0.0; % Default to 0 start
    end
    if nargin < 5
        n_iter = 60; % Default iterations (increased for better convergence)
    end
    if nargin < 6
        params_file = ''; % No params file by default
    end

    fprintf('=== PID Gain Optimization ===\n');
    fprintf('Track: %s (radius=%.1fm)\n', track_type, track_radius);
    fprintf('Simulation time: %.1fs\n', sim_time);
    fprintf('Initial speed: %.1f m/s\n', initial_speed);
    fprintf('Iterations: %d\n\n', n_iter);

    % トラックタイプに応じたファイルパス設定
    if strcmp(track_type, 'ASE_autocross')
        track_file_arg = '/workspace/data/ASE_autocross_sim.csv';
    else
        track_file_arg = track_type;
    end

    % Docker PATHの設定（バッチモード対応）
    current_path = getenv('PATH');
    if ismac
        docker_dir = '/opt/homebrew/bin';
        if ~contains(current_path, docker_dir)
            setenv('PATH', [docker_dir, ':', current_path]);
        end
    elseif ispc
        docker_dir = 'C:\Program Files\Docker\Docker\resources\bin';
        if ~contains(current_path, docker_dir)
            setenv('PATH', [docker_dir, ';', current_path]);
        end
    end

    % パス設定
    project_root = fileparts(pwd);

    % VehicleSimRunnerを使用して永続的なDockerコンテナを確保
    runner = VehicleSimRunner(project_root);

    % params_fileオプション（コンテナ内パス）
    if ~isempty(params_file)
        base_extra_args = sprintf('--params_file %s', params_file);
    else
        base_extra_args = '';
    end

    % バッチモード検出（-batch実行時はプロットを無効化）
    is_batch_mode = ~usejava('desktop');

    %% 最適化変数の定義
    % トラック別の検索範囲（既知の良好なゲイン付近に絞る）
    if strcmp(track_type, 'ASE_autocross')
        % ASE Autocross用: 完走 + 速度最大化を両立するための調整済み範囲
        Kp_steer  = optimizableVariable('Kp_steer',  [0.08, 0.60], 'Type', 'real');
        Kd_steer  = optimizableVariable('Kd_steer',  [0.05, 0.40], 'Type', 'real');
        Ki_steer  = optimizableVariable('Ki_steer',  [0.00, 0.05], 'Type', 'real');
        Kh_steer  = optimizableVariable('Kh_steer',  [0.20, 1.20], 'Type', 'real');
        Kff_steer = optimizableVariable('Kff_steer', [0.50, 1.50], 'Type', 'real');
        Kp_speed  = optimizableVariable('Kp_speed',  [0.30, 1.50], 'Type', 'real');
        Ki_speed  = optimizableVariable('Ki_speed',  [0.00, 0.10], 'Type', 'real');
        brake_gain_ratio = optimizableVariable('brake_gain_ratio', [0.05, 0.30], 'Type', 'real');
        corner_thr_red   = optimizableVariable('corner_thr_red',   [0.20, 0.70], 'Type', 'real');
    else
        % その他のトラック用: 広い検索範囲
        Kp_steer  = optimizableVariable('Kp_steer',  [0.1, 2.0], 'Type', 'real');
        Kd_steer  = optimizableVariable('Kd_steer',  [0.05, 1.0], 'Type', 'real');
        Ki_steer  = optimizableVariable('Ki_steer',  [0.0, 0.2], 'Type', 'real');
        Kh_steer  = optimizableVariable('Kh_steer',  [0.1, 1.5], 'Type', 'real');
        Kff_steer = optimizableVariable('Kff_steer', [0.5, 2.0], 'Type', 'real');
        Kp_speed  = optimizableVariable('Kp_speed',  [0.1, 1.0], 'Type', 'real');
        Ki_speed  = optimizableVariable('Ki_speed',  [0.0, 0.3], 'Type', 'real');
    end

    if strcmp(track_type, 'ASE_autocross')
        vars = [Kp_steer, Kd_steer, Ki_steer, Kh_steer, Kff_steer, Kp_speed, Ki_speed, brake_gain_ratio, corner_thr_red];
    else
        vars = [Kp_steer, Kd_steer, Ki_steer, Kh_steer, Kff_steer, Kp_speed, Ki_speed];
    end

    %% トラック別の初期シード（動作確認済みゲインをInitialXとして渡す）
    if strcmp(track_type, 'ASE_autocross')
        initial_x = table(0.109723, 0.153417, 0.024839, 0.715947, 1.069527, 0.800000, 0.020000, 0.10, 0.45, ...
            'VariableNames', {'Kp_steer','Kd_steer','Ki_steer','Kh_steer','Kff_steer','Kp_speed','Ki_speed','brake_gain_ratio','corner_thr_red'});
    elseif strcmp(track_type, 'figure_eight')
        initial_x = table(1.992569, 0.057854, 0.064521, 0.923226, 0.588280, 0.767312, 0.111730, ...
            'VariableNames', {'Kp_steer','Kd_steer','Ki_steer','Kh_steer','Kff_steer','Kp_speed','Ki_speed'});
    elseif strcmp(track_type, 'skidpad')
        initial_x = table(1.5, 0.15, 0.05, 0.6, 1.0, 0.5, 0.1, ...
            'VariableNames', {'Kp_steer','Kd_steer','Ki_steer','Kh_steer','Kff_steer','Kp_speed','Ki_speed'});
    else
        % デフォルト: 中程度のゲイン
        initial_x = table(0.8, 0.3, 0.05, 0.5, 1.0, 0.3, 0.05, ...
            'VariableNames', {'Kp_steer','Kd_steer','Ki_steer','Kh_steer','Kff_steer','Kp_speed','Ki_speed'});
    end

    %% 評価関数
    function cost = objective_function(params)
        % パラメータ抽出
        kp_st = params.Kp_steer;
        kd_st = params.Kd_steer;
        ki_st = params.Ki_steer;
        kh_st = params.Kh_steer;
        kff_st = params.Kff_steer;
        kp_sp = params.Kp_speed;
        ki_sp = params.Ki_speed;
        max_th = 1.0;

        % PIDゲインのCLI引数を構築
        gain_args = sprintf(['--kp-steer %.6f --kd-steer %.6f --ki-steer %.6f ' ...
            '--kh-steer %.6f --kff-steer %.6f ' ...
            '--kp-speed %.6f --ki-speed %.6f --max-throttle %.6f'], ...
            kp_st, kd_st, ki_st, kh_st, kff_st, kp_sp, ki_sp, max_th);

        % ASE_autocrossのみbrake_gain_ratioとcorner_thr_redを追加パラメータとして最適化
        if isfield(params, 'brake_gain_ratio')
            bgr = params.brake_gain_ratio;
            ctr = params.corner_thr_red;
            gain_args = [gain_args sprintf(' --brake-gain-ratio %.6f --corner-throttle-reduction %.6f', bgr, ctr)];
        end

        % base_extra_args（params_file等）を結合
        if ~isempty(base_extra_args)
            extra_args = [gain_args ' ' base_extra_args];
        else
            extra_args = gain_args;
        end

        % VehicleSimRunnerで実行（永続コンテナ＋JSON直接取得）
        [sim_result, status_ok] = runner.run([], track_file_arg, track_radius, sim_time, ...
            'extra_cli_args', extra_args, 'initial_speed', initial_speed);

        % Abort only when there is no usable result (crash/parse failure).
        % Incomplete laps (lap_completed=false) still carry partial data and
        % will be penalised by the DNF block below, preserving gradient info.
        if isempty(sim_result)
            cost = 1e6;
            fprintf('  [FAILED] No result from simulation (crash or parse error)\n');
            return;
        end

        % JSONフィールドから指標を取得
        max_dist      = sim_result.distance_traveled;
        max_lateral   = sim_result.max_lateral_error;
        avg_lat       = sim_result.avg_lateral_error;
        avg_speed     = sim_result.avg_speed;
        rms_heading   = sim_result.rms_heading_error;
        steer_smooth  = sim_result.steering_smoothness;
        thr_smooth    = sim_result.throttle_smoothness;

        % max_lateralが大きすぎる場合は距離をリセット（コースアウト判定）
        if max_lateral > 12.0
            max_dist = min(max_dist, 10.0);
        end

        % Set target distance based on track type
        if strcmp(track_type, 'ASE_autocross') || contains(track_type, '.csv')
            target_dist = 1000.0;
        elseif strcmp(track_type, 'acceleration')
            target_dist = track_radius;
        else
            target_dist = 2 * pi * track_radius;
            if strcmp(track_type, 'figure_eight')
                target_dist = target_dist * 2;
            end
        end

        % コスト計算
        % avg_lateral_error はRMS横偏差の近似として使用
        rms_lateral = avg_lat;

        w_lat       = 3.0;
        w_head      = 0.8;
        w_smooth_s  = 6.0;
        w_smooth_t  = 0.3;
        w_max_lat   = 12.0;
        w_max_head  = 0.3;
        w_avg_speed = 1.5;
        ref_speed   = 22.0;

        cost = w_lat * rms_lateral + ...
               w_head * rms_heading + ...
               w_smooth_s * steer_smooth + ...
               w_smooth_t * thr_smooth + ...
               w_max_lat * max_lateral + ...
               w_max_head * 0 + ...
               w_avg_speed * max(0, ref_speed - avg_speed);

        % DNF/失敗判定: lap_completed が false かつ距離不足の場合
        is_failure = isnan(cost) || isinf(cost) || (~sim_result.lap_completed && max_dist < target_dist * 0.95);

        if is_failure
            if isnan(cost) || isinf(cost)
                fprintf('  [NaN/Inf at dist=%.0fm] ', max_dist);
            else
                fprintf('  [DNF] ');
            end
            base_failure_cost = 20000.0;
            dist_factor = 200.0;
            dist_penalty = max(0, (target_dist - max_dist) * dist_factor);
            cost = base_failure_cost + dist_penalty;
            fprintf('Dist=%.1fm Cost->%.1f ', max_dist, cost);
        end

        fprintf('  Cost=%.2f | Lat=%.3fm | Head=%.2fdeg | SteerSmooth=%.4f | AvgSpd=%.1fm/s\n', ...
                cost, rms_lateral, rms_heading, steer_smooth, avg_speed);
    end

    %% ベイズ最適化実行
    fprintf('Starting Bayesian Optimization...\n');
    fprintf('This may take 15-30 minutes depending on MaxObjectiveEvaluations\n\n');

    if is_batch_mode
        plot_fcns = [];
    else
        plot_fcns = {@plotObjectiveModel, @plotMinObjective};
    end

    results = bayesopt(@objective_function, vars, ...
        'MaxObjectiveEvaluations', n_iter, ...
        'IsObjectiveDeterministic', true, ...
        'AcquisitionFunctionName', 'expected-improvement-plus', ...
        'InitialX', initial_x, ...
        'PlotFcn', plot_fcns, ...
        'Verbose', 1, ...
        'UseParallel', false);

    %% 最適解の表示
    fprintf('\n=== Optimization Complete! ===\n');
    fprintf('Best cost: %.4f\n', results.MinObjective);
    fprintf('\nOptimal PID Gains:\n');
    fprintf('  Kp_steer  = %.6f\n', results.XAtMinObjective.Kp_steer);
    fprintf('  Kd_steer  = %.6f\n', results.XAtMinObjective.Kd_steer);
    fprintf('  Ki_steer  = %.6f\n', results.XAtMinObjective.Ki_steer);
    fprintf('  Kh_steer  = %.6f\n', results.XAtMinObjective.Kh_steer);
    fprintf('  Kff_steer = %.6f\n', results.XAtMinObjective.Kff_steer);
    fprintf('  Kp_speed  = %.6f\n', results.XAtMinObjective.Kp_speed);
    fprintf('  Ki_speed  = %.6f\n', results.XAtMinObjective.Ki_speed);
    fprintf('  max_throttle = 1.0 (Fixed)\n');
    if isfield(results.XAtMinObjective, 'brake_gain_ratio')
        fprintf('  brake_gain_ratio       = %.6f\n', results.XAtMinObjective.brake_gain_ratio);
        fprintf('  corner_thr_red         = %.6f\n', results.XAtMinObjective.corner_thr_red);
    end

    %% 最適ゲインで最終シミュレーション実行
    fprintf('\nRunning final simulation with optimal gains...\n');
    best = results.XAtMinObjective;

    best_gain_args = sprintf(['--kp-steer %.6f --kd-steer %.6f --ki-steer %.6f ' ...
        '--kh-steer %.6f --kff-steer %.6f ' ...
        '--kp-speed %.6f --ki-speed %.6f --max-throttle %.6f'], ...
        best.Kp_steer, best.Kd_steer, best.Ki_steer, ...
        best.Kh_steer, best.Kff_steer, best.Kp_speed, best.Ki_speed, 1.0);
    if isfield(best, 'brake_gain_ratio')
        best_gain_args = [best_gain_args sprintf(' --brake-gain-ratio %.6f --corner-throttle-reduction %.6f', ...
            best.brake_gain_ratio, best.corner_thr_red)];
    end
    if ~isempty(base_extra_args)
        best_gain_args = [best_gain_args ' ' base_extra_args];
    end

    % 最終シミュレーションはCSV出力も生成（可視化用）
    final_sim_cmd = sprintf(['bash -c "cd build && ' ...
        './cpp/laptime_sim --track %s --radius %.1f --time %.1f --initial-speed %.1f ' ...
        '%s --output optimized_result.csv && cp optimized_result.csv /workspace/matlab/optimized_result.csv"'], ...
        track_file_arg, track_radius, sim_time * 1.5, initial_speed, best_gain_args);

    project_root_for_docker = fileparts(pwd);
    docker_cmd = sprintf('docker compose -f %s/docker-compose.yml exec -T fsae-sim', project_root_for_docker);
    system(sprintf('%s %s', docker_cmd, final_sim_cmd));

    %% 結果の可視化
    if ~is_batch_mode
        fprintf('Visualizing optimized results...\n');
        visualize_laptime_sim('optimized_result.csv', track_type, track_radius);
    else
        fprintf('Batch mode: Skipping interactive visualization.\n');
    end

    %% 結果をテキストファイルに保存
    output_file = sprintf('optimal_gains_%s.txt', track_type);
    fid = fopen(output_file, 'w');
    fprintf(fid, '=== Optimal PID Gains for %s Track ===\n', track_type);
    fprintf(fid, 'Track radius: %.1f m\n', track_radius);
    fprintf(fid, 'Optimization date: %s\n\n', datetime('now'));
    fprintf(fid, '# C++ Command Line Arguments:\n');
    fprintf(fid, '--kp-steer %.6f --kd-steer %.6f --ki-steer %.6f \\\n', ...
            best.Kp_steer, best.Kd_steer, best.Ki_steer);
    fprintf(fid, '--kh-steer %.6f --kff-steer %.6f \\\n', ...
            best.Kh_steer, best.Kff_steer);
    fprintf(fid, '--kp-speed %.6f --ki-speed %.6f \\\n', ...
            best.Kp_speed, best.Ki_speed);
    fprintf(fid, '--max-throttle 1.0\n\n');
    fprintf(fid, '# Individual Gains:\n');
    fprintf(fid, 'Kp_steer  = %.6f\n', best.Kp_steer);
    fprintf(fid, 'Kd_steer  = %.6f\n', best.Kd_steer);
    fprintf(fid, 'Ki_steer  = %.6f\n', best.Ki_steer);
    fprintf(fid, 'Kh_steer  = %.6f\n', best.Kh_steer);
    fprintf(fid, 'Kff_steer = %.6f\n', best.Kff_steer);
    fprintf(fid, 'Kp_speed  = %.6f\n', best.Kp_speed);
    fprintf(fid, 'Ki_speed  = %.6f\n', best.Ki_speed);
    fprintf(fid, 'max_throttle = 1.0 (Fixed)\n');
    if isfield(best, 'brake_gain_ratio')
        fprintf(fid, 'brake_gain_ratio = %.6f\n', best.brake_gain_ratio);
        fprintf(fid, 'corner_thr_red   = %.6f\n', best.corner_thr_red);
    end
    fprintf(fid, '\nBest cost: %.4f\n', results.MinObjective);
    fclose(fid);

    fprintf('\nOptimal gains saved to: %s\n', output_file);
    fprintf('Done!\n');
end
