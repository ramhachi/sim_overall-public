%% シミュレーション結果比較スクリプト
% デフォルトゲイン vs 最適化ゲインの比較
%
% 使い方:
%   compare_results('default_sim.csv', 'optimized_sim.csv', 'figure_eight', 25)

function compare_results(default_csv, optimized_csv, track_type, track_radius)
    if nargin < 1
        default_csv = 'default_sim.csv';
    end
    if nargin < 2
        optimized_csv = 'optimized_sim.csv';
    end
    if nargin < 3
        track_type = 'figure_eight';
    end
    if nargin < 4
        track_radius = 25.0;
    end

    fprintf('=== Comparing Simulation Results ===\n');
    fprintf('Default:   %s\n', default_csv);
    fprintf('Optimized: %s\n\n', optimized_csv);

    % データ読み込み
    data_default = readtable(default_csv);
    data_opt = readtable(optimized_csv);

    % トラック生成
    [track_x, track_y] = generate_track_centerline(track_type, track_radius);

    % メインフィギュア
    figure('Name', 'Default vs Optimized Comparison', ...
           'Position', [50 50 1800 1000], 'Color', 'w');

    %% 1. 車両軌跡比較
    subplot(2, 4, [1 2 5 6])
    hold on; grid on; axis equal;

    % トラック
    plot(track_x, track_y, 'k--', 'LineWidth', 2.5, 'DisplayName', 'Track');

    % デフォルト軌跡
    plot(data_default.x, data_default.y, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Default');

    % 最適化軌跡
    plot(data_opt.x, data_opt.y, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Optimized');

    % スタート/ゴール
    plot(data_default.x(1), data_default.y(1), 'go', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Start');

    xlabel('X [m]', 'FontSize', 12);
    ylabel('Y [m]', 'FontSize', 12);
    title('Vehicle Trajectory Comparison', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 10);
    xlim([min([data_default.x; data_opt.x])-5, max([data_default.x; data_opt.x])+5]);
    ylim([min([data_default.y; data_opt.y])-5, max([data_default.y; data_opt.y])+5]);

    %% 2. 横偏差比較
    subplot(2, 4, 3)
    hold on; grid on;
    plot(data_default.time, abs(data_default.lateral_error), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Default');
    plot(data_opt.time, abs(data_opt.lateral_error), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Optimized');
    xlabel('Time [s]', 'FontSize', 10);
    ylabel('|Lateral Error| [m]', 'FontSize', 10);
    title('Lateral Tracking Error', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best');
    ylim([0 max([abs(data_default.lateral_error); abs(data_opt.lateral_error)])*1.1]);

    %% 3. ヘディング誤差比較
    subplot(2, 4, 4)
    hold on; grid on;
    plot(data_default.time, abs(data_default.heading_error_deg), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Default');
    plot(data_opt.time, abs(data_opt.heading_error_deg), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Optimized');
    xlabel('Time [s]', 'FontSize', 10);
    ylabel('|Heading Error| [deg]', 'FontSize', 10);
    title('Heading Error', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best');

    %% 4. ステアリング角比較
    subplot(2, 4, 7)
    hold on; grid on;
    plot(data_default.time, rad2deg(data_default.steering), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Default');
    plot(data_opt.time, rad2deg(data_opt.steering), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Optimized');
    xlabel('Time [s]', 'FontSize', 10);
    ylabel('Steering Angle [deg]', 'FontSize', 10);
    title('Steering Input', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best');

    %% 5. 速度比較
    subplot(2, 4, 8)
    hold on; grid on;
    plot(data_default.time, data_default.speed, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Default');
    plot(data_opt.time, data_opt.speed, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Optimized');
    xlabel('Time [s]', 'FontSize', 10);
    ylabel('Speed [m/s]', 'FontSize', 10);
    title('Vehicle Speed', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best');

    %% 統計比較
    fprintf('\n=== Performance Comparison ===\n\n');

    fprintf('%-30s | %15s | %15s | %10s\n', 'Metric', 'Default', 'Optimized', 'Change');
    fprintf('%s\n', repmat('-', 1, 75));

    metrics = {
        'RMS Lateral Error [m]', sqrt(mean(data_default.lateral_error.^2)), sqrt(mean(data_opt.lateral_error.^2));
        'Max Lateral Error [m]', max(abs(data_default.lateral_error)), max(abs(data_opt.lateral_error));
        'RMS Heading Error [deg]', sqrt(mean(data_default.heading_error_deg.^2)), sqrt(mean(data_opt.heading_error_deg.^2));
        'Max Heading Error [deg]', max(abs(data_default.heading_error_deg)), max(abs(data_opt.heading_error_deg));
        'Avg Speed [m/s]', mean(data_default.speed), mean(data_opt.speed);
        'Max Speed [m/s]', max(data_default.speed), max(data_opt.speed);
        'Steering Smoothness', sqrt(mean(diff(data_default.steering).^2)), sqrt(mean(diff(data_opt.steering).^2));
    };

    for i = 1:size(metrics, 1)
        name = metrics{i, 1};
        val_def = metrics{i, 2};
        val_opt = metrics{i, 3};

        if val_def > 1e-6
            change_pct = (val_opt - val_def) / val_def * 100;
            change_str = sprintf('%+.1f%%', change_pct);

            % 改善/悪化の判定（速度以外は小さい方が良い）
            if contains(name, 'Speed') || contains(name, 'Avg')
                % 速度は高い方が良い場合もある（文脈による）
                marker = '';
            elseif change_pct < 0
                marker = ' ✅';  % 改善
            else
                marker = ' ❌';  % 悪化
            end

            fprintf('%-30s | %15.3f | %15.3f | %10s%s\n', ...
                    name, val_def, val_opt, change_str, marker);
        else
            fprintf('%-30s | %15.3f | %15.3f | %10s\n', ...
                    name, val_def, val_opt, 'N/A');
        end
    end

    fprintf('\n');

    %% 追加フィギュア：誤差ヒストグラム
    figure('Name', 'Error Distribution', 'Position', [100 100 1200 500], 'Color', 'w');

    % 横偏差ヒストグラム
    subplot(1, 2, 1)
    hold on; grid on;
    histogram(data_default.lateral_error, 30, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'DisplayName', 'Default');
    histogram(data_opt.lateral_error, 30, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'DisplayName', 'Optimized');
    xlabel('Lateral Error [m]', 'FontSize', 11);
    ylabel('Frequency', 'FontSize', 11);
    title('Lateral Error Distribution', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best');

    % ヘディング誤差ヒストグラム
    subplot(1, 2, 2)
    hold on; grid on;
    histogram(data_default.heading_error_deg, 30, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'DisplayName', 'Default');
    histogram(data_opt.heading_error_deg, 30, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'DisplayName', 'Optimized');
    xlabel('Heading Error [deg]', 'FontSize', 11);
    ylabel('Frequency', 'FontSize', 11);
    title('Heading Error Distribution', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best');

    fprintf('Comparison complete!\n');
end

%% トラック中心線生成（C++のtrack_model.hppに合わせて修正）
function [x, y] = generate_track_centerline(track_type, radius)
    switch track_type
        case 'circle'
            kappa = 1.0 / radius;
            circumference = 2 * pi * radius;
            n_points = 500;
            s_vals = linspace(0, circumference, n_points);
            x = zeros(size(s_vals));
            y = zeros(size(s_vals));
            theta = 0;
            x(1) = 0;
            y(1) = 0;
            for i = 2:length(s_vals)
                ds = s_vals(i) - s_vals(i-1);
                theta = theta + kappa * ds;
                x(i) = x(i-1) + cos(theta) * ds;
                y(i) = y(i-1) + sin(theta) * ds;
            end

        case 'figure_eight'
            kappa1 = 1.0 / radius;
            kappa2 = -1.0 / radius;
            half_circ = pi * radius;
            n_half = 250;
            s1 = linspace(0, half_circ, n_half);
            x1 = zeros(size(s1));
            y1 = zeros(size(s1));
            theta1 = 0;
            x1(1) = 0;
            y1(1) = 0;
            for i = 2:length(s1)
                ds = s1(i) - s1(i-1);
                theta1 = theta1 + kappa1 * ds;
                x1(i) = x1(i-1) + cos(theta1) * ds;
                y1(i) = y1(i-1) + sin(theta1) * ds;
            end
            s2 = linspace(0, half_circ, n_half);
            x2 = zeros(size(s2));
            y2 = zeros(size(s2));
            theta2 = theta1;
            x2(1) = x1(end);
            y2(1) = y1(end);
            for i = 2:length(s2)
                ds = s2(i) - s2(i-1);
                theta2 = theta2 + kappa2 * ds;
                x2(i) = x2(i-1) + cos(theta2) * ds;
                y2(i) = y2(i-1) + sin(theta2) * ds;
            end
            x = [x1, x2(2:end)];
            y = [y1, y2(2:end)];

        case 'straight_corner'
            straight_length = 50.0;
            corner_arc = pi * radius;
            n_straight = 200;
            n_corner = 200;
            s_straight = linspace(0, straight_length, n_straight);
            x_straight = s_straight;
            y_straight = zeros(size(s_straight));
            kappa = 1.0 / radius;
            s_corner = linspace(0, corner_arc, n_corner);
            x_corner = zeros(size(s_corner));
            y_corner = zeros(size(s_corner));
            theta = 0;
            x_corner(1) = straight_length;
            y_corner(1) = 0;
            for i = 2:length(s_corner)
                ds = s_corner(i) - s_corner(i-1);
                theta = theta + kappa * ds;
                x_corner(i) = x_corner(i-1) + cos(theta) * ds;
                y_corner(i) = y_corner(i-1) + sin(theta) * ds;
            end
            x = [x_straight, x_corner(2:end)];
            y = [y_straight, y_corner(2:end)];

        otherwise
            error('Unknown track type: %s', track_type);
    end
end
