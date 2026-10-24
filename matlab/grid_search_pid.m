%% PIDゲイン グリッドサーチ（軽量版）
% 主要なゲインのみをグリッドサーチで探索
% VehicleSimRunnerを使用して永続的なDockerコンテナで実行し、
% JSONで結果を直接取得することでCSV読み書きのオーバーヘッドを削減。
%
% 使い方:
%   cd <project_root>/matlab
%   best_gains = grid_search_pid('figure_eight', 25, 15)

function best_gains = grid_search_pid(track_type, track_radius, sim_time)
    % デフォルト引数
    if nargin < 1
        track_type = 'figure_eight';
    end
    if nargin < 2
        track_radius = 25.0;
    end
    if nargin < 3
        sim_time = 10.0;  % グリッドサーチは短めに
    end

    fprintf('=== PID Gain Grid Search ===\n');
    fprintf('Track: %s (radius=%.1fm)\n', track_type, track_radius);
    fprintf('Simulation time: %.1fs\n\n', sim_time);

    % トラックタイプに応じたファイルパス設定
    if strcmp(track_type, 'ASE_autocross')
        track_file_arg = '/workspace/data/ASE_autocross_sim.csv';
    else
        track_file_arg = track_type;
    end

    % パス設定
    project_root = fileparts(pwd);

    % VehicleSimRunnerを使用して永続的なDockerコンテナを確保
    runner = VehicleSimRunner(project_root);

    %% グリッド定義（主要ゲインのみ）
    Kp_steer_range = [0.4, 0.6, 0.8, 1.0, 1.2];
    Kd_steer_range = [0.2, 0.3, 0.4, 0.5];
    Kh_steer_range = [0.3, 0.5, 0.7];

    Ki_steer_fixed = 0.05;
    Kff_steer_fixed = 1.0;
    Kp_speed_fixed = 0.3;
    Ki_speed_fixed = 0.05;

    total_iterations = length(Kp_steer_range) * length(Kd_steer_range) * length(Kh_steer_range);
    fprintf('Total combinations: %d\n', total_iterations);
    fprintf('Estimated time: %.1f minutes\n\n', total_iterations * sim_time / 60);

    results = [];
    iteration = 0;

    %% グリッドサーチ実行
    fprintf('Starting grid search...\n');
    tic;

    for kp_st = Kp_steer_range
        for kd_st = Kd_steer_range
            for kh_st = Kh_steer_range
                iteration = iteration + 1;

                fprintf('[%d/%d] Testing: Kp=%.2f, Kd=%.2f, Kh=%.2f ... ', ...
                        iteration, total_iterations, kp_st, kd_st, kh_st);

                % PIDゲインのCLI引数を構築
                gain_args = sprintf(['--kp-steer %.6f --kd-steer %.6f --ki-steer %.6f ' ...
                    '--kh-steer %.6f --kff-steer %.6f ' ...
                    '--kp-speed %.6f --ki-speed %.6f'], ...
                    kp_st, kd_st, Ki_steer_fixed, kh_st, Kff_steer_fixed, ...
                    Kp_speed_fixed, Ki_speed_fixed);

                % VehicleSimRunnerで実行（永続コンテナ＋JSON直接取得）
                [sim_result, status_ok] = runner.run([], track_file_arg, track_radius, sim_time, ...
                    'extra_cli_args', gain_args);

                if ~status_ok || isempty(sim_result)
                    fprintf('FAILED (unstable)\n');
                    continue;
                end

                % JSONフィールドから指標を取得
                rms_lateral          = sim_result.avg_lateral_error;
                rms_heading          = sim_result.rms_heading_error;
                max_lateral          = sim_result.max_lateral_error;
                steering_smoothness  = sim_result.steering_smoothness;

                % コスト計算
                cost = 10.0 * rms_lateral + ...
                       0.5 * rms_heading + ...
                       5.0 * max_lateral + ...
                       2.0 * steering_smoothness;

                fprintf('Cost=%.2f (Lat=%.3fm)\n', cost, rms_lateral);

                result.Kp_steer = kp_st;
                result.Kd_steer = kd_st;
                result.Kh_steer = kh_st;
                result.cost = cost;
                result.rms_lateral = rms_lateral;
                result.rms_heading = rms_heading;
                result.max_lateral = max_lateral;
                results = [results; result]; %#ok<AGROW>
            end
        end
    end

    elapsed_time = toc;
    fprintf('\nGrid search complete! Total time: %.1f minutes\n', elapsed_time / 60);

    %% 最良結果の表示
    if isempty(results)
        error('No valid results found. All configurations failed.');
    end

    [~, best_idx] = min([results.cost]);
    best_gains = results(best_idx);

    fprintf('\n=== Best Configuration ===\n');
    fprintf('Cost: %.4f\n', best_gains.cost);
    fprintf('RMS Lateral Error: %.3f m\n', best_gains.rms_lateral);
    fprintf('RMS Heading Error: %.2f deg\n', best_gains.rms_heading);
    fprintf('Max Lateral Error: %.3f m\n\n', best_gains.max_lateral);
    fprintf('Optimal Gains:\n');
    fprintf('  Kp_steer  = %.6f\n', best_gains.Kp_steer);
    fprintf('  Kd_steer  = %.6f\n', best_gains.Kd_steer);
    fprintf('  Ki_steer  = %.6f (fixed)\n', Ki_steer_fixed);
    fprintf('  Kh_steer  = %.6f\n', best_gains.Kh_steer);
    fprintf('  Kff_steer = %.6f (fixed)\n', Kff_steer_fixed);
    fprintf('  Kp_speed  = %.6f (fixed)\n', Kp_speed_fixed);
    fprintf('  Ki_speed  = %.6f (fixed)\n', Ki_speed_fixed);

    %% 結果の可視化
    figure('Name', 'Grid Search Results', 'Position', [100 100 1200 800]);

    subplot(2, 2, 1);
    kp_unique = unique([results.Kp_steer]);
    kd_unique = unique([results.Kd_steer]);
    kh_unique = unique([results.Kh_steer]);

    kh_mid = kh_unique(ceil(length(kh_unique)/2));
    idx_kh = abs([results.Kh_steer] - kh_mid) < 0.01;
    subset = results(idx_kh);

    [KP, KD] = meshgrid(kp_unique, kd_unique);
    COST = zeros(size(KP));
    for i = 1:length(subset)
        r_i = find(abs(kp_unique - subset(i).Kp_steer) < 0.01);
        c_i = find(abs(kd_unique - subset(i).Kd_steer) < 0.01);
        COST(c_i, r_i) = subset(i).cost;
    end

    imagesc(kp_unique, kd_unique, COST);
    colorbar;
    xlabel('Kp_{steer}');
    ylabel('Kd_{steer}');
    title(sprintf('Cost Map (Kh_{steer}=%.2f)', kh_mid));
    hold on;
    plot(best_gains.Kp_steer, best_gains.Kd_steer, 'r*', 'MarkerSize', 15, 'LineWidth', 2);

    subplot(2, 2, 2);
    scatter([results.rms_lateral], [results.cost], 50, 'filled');
    xlabel('RMS Lateral Error [m]');
    ylabel('Cost');
    title('Cost vs Lateral Error');
    grid on;

    subplot(2, 2, 3);
    scatter([results.rms_lateral], [results.rms_heading], 50, [results.cost], 'filled');
    colorbar;
    xlabel('RMS Lateral Error [m]');
    ylabel('RMS Heading Error [deg]');
    title('Error Trade-off');
    grid on;

    subplot(2, 2, 4);
    plot([results.cost], 'b.-', 'LineWidth', 1.5);
    xlabel('Iteration');
    ylabel('Cost');
    title('Cost History');
    grid on;
    yline(best_gains.cost, 'r--', 'LineWidth', 2);
    legend('All trials', 'Best');

    %% 最適ゲインでシミュレーション実行（可視化用CSV生成）
    fprintf('\nRunning final simulation with best gains...\n');
    final_output = 'grid_optimized.csv';

    final_gain_args = sprintf(['--kp-steer %.6f --kd-steer %.6f --ki-steer %.6f ' ...
        '--kh-steer %.6f --kff-steer %.6f ' ...
        '--kp-speed %.6f --ki-speed %.6f'], ...
        best_gains.Kp_steer, best_gains.Kd_steer, Ki_steer_fixed, ...
        best_gains.Kh_steer, Kff_steer_fixed, Kp_speed_fixed, Ki_speed_fixed);

    final_cmd = sprintf(['bash -c "cd build && ' ...
        './cpp/laptime_sim --track %s --radius %.1f --time %.1f ' ...
        '%s --output %s && cp %s /workspace/matlab/%s"'], ...
        track_file_arg, track_radius, sim_time * 2, ...
        final_gain_args, final_output, final_output, final_output);

    project_root_for_docker = fileparts(pwd);
    docker_cmd = sprintf('docker compose -f %s/docker-compose.yml exec -T fsae-sim', project_root_for_docker);
    system(sprintf('%s %s', docker_cmd, final_cmd));
    visualize_laptime_sim('grid_optimized.csv', track_type, track_radius);

    %% 結果保存
    output_file = sprintf('grid_search_results_%s.txt', track_type);
    fid = fopen(output_file, 'w');
    fprintf(fid, '=== Grid Search Results for %s Track ===\n', track_type);
    fprintf(fid, 'Date: %s\n', datetime('now'));
    fprintf(fid, 'Total iterations: %d\n', total_iterations);
    fprintf(fid, 'Elapsed time: %.1f minutes\n\n', elapsed_time / 60);
    fprintf(fid, 'Best cost: %.4f\n', best_gains.cost);
    fprintf(fid, 'RMS Lateral Error: %.3f m\n', best_gains.rms_lateral);
    fprintf(fid, 'RMS Heading Error: %.2f deg\n\n', best_gains.rms_heading);
    fprintf(fid, '# Command line arguments:\n');
    fprintf(fid, '--kp-steer %.6f --kd-steer %.6f --ki-steer %.6f \\\n', ...
            best_gains.Kp_steer, best_gains.Kd_steer, Ki_steer_fixed);
    fprintf(fid, '--kh-steer %.6f --kff-steer %.6f \\\n', ...
            best_gains.Kh_steer, Kff_steer_fixed);
    fprintf(fid, '--kp-speed %.6f --ki-speed %.6f\n', ...
            Kp_speed_fixed, Ki_speed_fixed);
    fclose(fid);

    fprintf('\nResults saved to: %s\n', output_file);
    fprintf('Done!\n');
end

