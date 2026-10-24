%% ラップタイムシミュレーション結果可視化
% Phase 7b: フル過渡ラップタイムシミュレーションの結果をプロット
%
% 使い方:
%   cd matlab
%   visualize_laptime_sim('../build/laptime_sim_figure_eight.csv', 'figure_eight', 25)

function visualize_laptime_sim(csv_filename, track_type, track_radius)
    % デフォルト引数
    if nargin < 1
        csv_filename = 'laptime_sim_figure_eight.csv';
    end
    if nargin < 2
        track_type = 'figure_eight';
    end
    if nargin < 3
        track_radius = 25.0;
    end

    % CSVデータ読み込み
    fprintf('Loading data from: %s\n', csv_filename);
    if ~isfile(csv_filename)
        error('File not found: %s\nPlease run from matlab/ directory or specify full path.', csv_filename);
    end
    data = readtable(csv_filename);

    % データ抽出
    t = data.time;
    x = data.x;
    y = data.y;
    vx = data.vx;
    vy = data.vy;
    r = data.r;
    s = data.s;
    kappa = data.kappa;
    steering = data.steering;
    throttle = data.throttle;
    brake = data.brake;
    speed = data.speed;
    target_speed = data.target_speed;
    lateral_error = data.lateral_error;
    heading_error_deg = data.heading_error_deg;

    % DRSアクティブフラグ（列が存在しない旧CSVにも対応）
    if ismember('drs_active', data.Properties.VariableNames)
        drs_active = logical(data.drs_active);
    else
        drs_active = false(size(t));
    end

    % トラック中心線生成
    [track_x, track_y] = generate_track_centerline(track_type, track_radius);

    % バッチモード検出: -batch実行時はfigureを非表示にしてPNG保存のみ行う
    is_batch_mode = ~usejava('desktop');
    if is_batch_mode
        fig_visible = 'off';
    else
        fig_visible = 'on';
    end

    % メインフィギュア作成
    figure('Name', 'Lap Time Simulation Results', ...
           'Position', [100 100 1600 1000], ...
           'Color', 'w', 'Visible', fig_visible);

    %% 1. 車両軌跡とトラック (大きめ)
    subplot(3, 3, [1 2 4 5])
    hold on; grid on; axis equal;

    % トラック中心線
    plot(track_x, track_y, 'k--', 'LineWidth', 2, 'DisplayName', 'Track Centerline');

    % 車両軌跡（速度で色付け）
    scatter(x, y, 20, speed, 'filled', 'DisplayName', 'Vehicle Path');
    colormap(jet);
    cb = colorbar;
    ylabel(cb, 'Speed [m/s]', 'FontSize', 10);

    % DRS有効区間をシアンで強調表示
    if any(drs_active)
        scatter(x(drs_active), y(drs_active), 35, 'c', 'filled', ...
            'DisplayName', 'DRS Active', 'MarkerEdgeColor', 'b');
    end

    % スタート地点
    plot(x(1), y(1), 'go', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Start');

    % 終了地点
    plot(x(end), y(end), 'rs', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'End');

    xlabel('X [m]', 'FontSize', 12);
    ylabel('Y [m]', 'FontSize', 12);
    title('Vehicle Trajectory', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best');

    %% 2. 速度プロファイル
    subplot(3, 3, 3)
    hold on; grid on;
    % DRS有効区間を背景ハイライト
    if any(drs_active)
        drs_starts = find(diff([0; drs_active]) == 1);
        drs_ends   = find(diff([drs_active; 0]) == -1);
        y_max = max(speed) * 1.1;
        for k = 1:length(drs_starts)
            patch([t(drs_starts(k)) t(drs_ends(k)) t(drs_ends(k)) t(drs_starts(k))], ...
                  [0 0 y_max y_max], 'c', 'FaceAlpha', 0.2, 'EdgeColor', 'none', ...
                  'HandleVisibility', 'off');
        end
        % 凡例用ダミー
        patch(NaN, NaN, 'c', 'FaceAlpha', 0.4, 'EdgeColor', 'none', 'DisplayName', 'DRS Active');
    end
    plot(t, speed, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Speed');
    plot(t, target_speed, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Target Speed');
    xlabel('Time [s]', 'FontSize', 10);
    ylabel('Speed [m/s]', 'FontSize', 10);
    title('Speed Profile', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best');
    ylim([0 max(speed)*1.1]);

    %% 3. 横偏差
    subplot(3, 3, 6)
    hold on; grid on;
    plot(t, lateral_error, 'b-', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 1);
    xlabel('Time [s]', 'FontSize', 10);
    ylabel('Lateral Error [m]', 'FontSize', 10);
    title('Lateral Error (n)', 'FontSize', 11, 'FontWeight', 'bold');

    %% 4. ヘディング誤差
    subplot(3, 3, 9)
    hold on; grid on;
    plot(t, heading_error_deg, 'r-', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 1);
    xlabel('Time [s]', 'FontSize', 10);
    ylabel('Heading Error [deg]', 'FontSize', 10);
    title('Heading Error', 'FontSize', 11, 'FontWeight', 'bold');

    %% 5. ステアリング角
    subplot(3, 3, 7)
    hold on; grid on;
    plot(t, rad2deg(steering), 'g-', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 1);
    xlabel('Time [s]', 'FontSize', 10);
    ylabel('Steering Angle [deg]', 'FontSize', 10);
    title('Steering Input', 'FontSize', 11, 'FontWeight', 'bold');

    %% 6. スロットル/ブレーキ
    subplot(3, 3, 8)
    hold on; grid on;
    area(t, throttle, 'FaceColor', 'g', 'FaceAlpha', 0.3, 'DisplayName', 'Throttle');
    area(t, -brake, 'FaceColor', 'r', 'FaceAlpha', 0.3, 'DisplayName', 'Brake');
    xlabel('Time [s]', 'FontSize', 10);
    ylabel('Input [-]', 'FontSize', 10);
    title('Throttle / Brake', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best');
    ylim([-1.1 1.1]);

    %% 統計表示
    fprintf('\n=== Simulation Statistics ===\n');
    fprintf('Duration: %.2f s\n', t(end));
    fprintf('Max Speed: %.2f m/s (%.1f km/h)\n', max(speed), max(speed)*3.6);
    fprintf('Avg Speed: %.2f m/s (%.1f km/h)\n', mean(speed), mean(speed)*3.6);
    fprintf('Max Lateral Error: %.3f m\n', max(abs(lateral_error)));
    fprintf('RMS Lateral Error: %.3f m\n', sqrt(mean(lateral_error.^2)));
    fprintf('Max Heading Error: %.2f deg\n', max(abs(heading_error_deg)));
    fprintf('RMS Heading Error: %.2f deg\n', sqrt(mean(heading_error_deg.^2)));
    fprintf('Max Steering Angle: %.2f deg\n', max(abs(rad2deg(steering))));
    fprintf('Distance Traveled: %.2f m\n', max(s));
    % DRS統計
    if any(drs_active)
        dt_data = mean(diff(t));
        drs_duration = sum(drs_active) * dt_data;
        drs_fraction_actual = drs_duration / t(end) * 100;
        fprintf('DRS Active: %.2f s (%.1f%% of lap)\n', drs_duration, drs_fraction_actual);
    else
        fprintf('DRS Active: 0 s (not available in this CSV)\n');
    end

    % 図1を保存
    [~, csv_base, ~] = fileparts(csv_filename);
    fig1_filename = sprintf('%s_main.png', csv_base);
    saveas(gcf, fig1_filename);
    fprintf('Saved figure 1 to: %s\n', fig1_filename);

    %% 追加フィギュア: 詳細時系列データ
    figure('Name', 'Detailed Time Series', ...
           'Position', [150 150 1400 900], ...
           'Color', 'w', 'Visible', fig_visible);

    % 車体速度成分
    subplot(3, 2, 1)
    hold on; grid on;
    plot(t, vx, 'b-', 'LineWidth', 1.5, 'DisplayName', 'v_x');
    plot(t, vy, 'r-', 'LineWidth', 1.5, 'DisplayName', 'v_y');
    xlabel('Time [s]'); ylabel('Velocity [m/s]');
    title('Body Velocities', 'FontWeight', 'bold');
    legend('Location', 'best');

    % ヨーレート
    subplot(3, 2, 2)
    hold on; grid on;
    plot(t, rad2deg(r), 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Yaw Rate [deg/s]');
    title('Yaw Rate', 'FontWeight', 'bold');

    % 弧長
    subplot(3, 2, 3)
    hold on; grid on;
    plot(t, s, 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Arc Length s [m]');
    title('Distance Along Track', 'FontWeight', 'bold');

    % 曲率
    subplot(3, 2, 4)
    hold on; grid on;
    plot(s, kappa, 'b.', 'MarkerSize', 3);
    xlabel('Arc Length s [m]'); ylabel('Curvature κ [1/m]');
    title('Track Curvature', 'FontWeight', 'bold');

    % 速度誤差
    subplot(3, 2, 5)
    hold on; grid on;
    speed_error = target_speed - speed;
    plot(t, speed_error, 'r-', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 1);
    xlabel('Time [s]'); ylabel('Speed Error [m/s]');
    title('Speed Tracking Error', 'FontWeight', 'bold');

    % 横偏差 vs 弧長
    subplot(3, 2, 6)
    hold on; grid on;
    plot(s, lateral_error, 'b.', 'MarkerSize', 3);
    yline(0, 'k--', 'LineWidth', 1);
    xlabel('Arc Length s [m]'); ylabel('Lateral Error [m]');
    title('Lateral Error vs Distance', 'FontWeight', 'bold');

    % 図2を保存
    fig2_filename = sprintf('%s_timeseries.png', csv_base);
    saveas(gcf, fig2_filename);
    fprintf('Saved figure 2 to: %s\n', fig2_filename);

    %% 追加フィギュア: g-gダイアグラム
    figure('Name', 'g-g Diagram', ...
           'Position', [200 200 800 700], ...
           'Color', 'w', 'Visible', fig_visible);

    % 加速度計算（数値微分）
    ax = gradient(vx) ./ gradient(t);  % 縦加速度
    ay = gradient(vy) ./ gradient(t) + vx .* r;  % 横加速度（遠心力含む）

    % g-gダイアグラム
    hold on; grid on; axis equal;
    scatter(ay/9.81, ax/9.81, 20, speed, 'filled');
    colormap(jet);
    cb = colorbar;
    ylabel(cb, 'Speed [m/s]');

    % 理論限界円
    theta = linspace(0, 2*pi, 100);
    r_lat = 1.5;  % 最大横加速度 [G]
    r_lon_accel = 1.0;  % 最大縦加速度 [G]
    r_lon_brake = 1.8;  % 最大制動加速度 [G]

    % 楕円近似の限界線
    for i = 1:length(theta)
        if cos(theta(i)) > 0
            ax_lim = r_lon_accel * cos(theta(i));
        else
            ax_lim = -r_lon_brake * cos(theta(i));
        end
        ay_lim = r_lat * sin(theta(i));
        plot(ay_lim, ax_lim, 'r-', 'LineWidth', 2);
    end

    xlabel('Lateral Acceleration a_y [G]', 'FontSize', 12);
    ylabel('Longitudinal Acceleration a_x [G]', 'FontSize', 12);
    title('g-g Diagram', 'FontSize', 14, 'FontWeight', 'bold');
    xlim([-4.5 4.5]); ylim([-4 4]);
    axis equal;


    % 図3を保存
    fig3_filename = sprintf('%s_gg.png', csv_base);
    saveas(gcf, fig3_filename);
    fprintf('Saved figure 3 to: %s\n', fig3_filename);

    fprintf('\nVisualization complete!\n');
end

%% トラック中心線生成（C++のtrack_model.hppに合わせて修正）
function [x, y] = generate_track_centerline(track_type, radius)
    switch track_type
        case 'circle'
            % 円形トラック：解析的に計算（原点中心、半径R）
            % C++のcreate_circle_trackと一致: 開始(R,0)、北向き(π/2)
            n_points = 500;
            theta_vals = linspace(0, 2*pi, n_points);
            x = radius * cos(theta_vals);
            y = radius * sin(theta_vals);

        case 'figure_eight'
            % 8の字トラック（真の縦並び2円版）
            % 上半円: 中心(0, R)、下半円: 中心(0, -R)
            % スタート: (R, 0)、北向き
            % 自己交差点: (0, 0)
            half_circumference = pi * radius;
            n_points = 500;
            
            % 弧長パラメータ
            s_vals = linspace(0, 2*half_circumference, n_points);
            x = zeros(size(s_vals));
            y = zeros(size(s_vals));
            
            for i = 1:n_points
                s = s_vals(i);
                
                if s <= half_circumference
                    % 上半円: 中心(0, R)
                    % angle: 0 → π
                    angle = s / radius;
                    x(i) = radius * cos(angle);
                    y(i) = radius + radius * sin(angle);
                else
                    % 下半円: 中心(0, -R)
                    % angle: π → 2π
                    s2 = s - half_circumference;
                    angle = s2 / radius + pi;
                    x(i) = radius * cos(angle);
                    y(i) = -radius + radius * sin(angle);
                end
            end
            
            % 閉ループ補正
            x(end) = x(1);
            y(end) = y(1);

        case 'skidpad'
            % Formula SAE スキッドパッド（水平figure-8）
            % 右円: 中心(+R, 0)、時計回り (κ=-1/R)
            % 左円: 中心(-R, 0)、反時計回り (κ=+1/R)
            % スタート: (0, 0)、北向き
            % トラック全長: 4πR
            full_circ = 2 * pi * radius;
            n_points = 500;
            
            s_vals = linspace(0, 2*full_circ, n_points);
            x = zeros(size(s_vals));
            y = zeros(size(s_vals));
            
            for i = 1:n_points
                s = s_vals(i);
                
                if s <= full_circ
                    % 右円: 中心(+R, 0)、CW
                    alpha = pi - s / radius;
                    x(i) = radius + radius * cos(alpha);
                    y(i) = radius * sin(alpha);
                else
                    % 左円: 中心(-R, 0)、CCW
                    s2 = s - full_circ;
                    alpha = s2 / radius;
                    x(i) = -radius + radius * cos(alpha);
                    y(i) = radius * sin(alpha);
                end
            end
            
            % 閉ループ補正
            x(end) = x(1);
            y(end) = y(1);

        case 'straight_corner'
            % 直線+コーナー
            straight_length = 50.0;
            corner_arc = pi * radius;
            n_straight = 200;
            n_corner = 200;

            % 直線部
            s_straight = linspace(0, straight_length, n_straight);
            x_straight = s_straight;
            y_straight = zeros(size(s_straight));

            % コーナー部
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

        case 'ASE_autocross'
            % Load from CSV
            % Assumes running from matlab/ directory
            data_file = fullfile(fileparts(pwd), 'data', 'ASE_autocross_sim.csv');
            if isfile(data_file)
                track_data = readtable(data_file);
                x = track_data.x;
                y = track_data.y;
            else
                warning('Track data file not found: %s', data_file);
                x = []; y = [];
            end

        otherwise
            error('Unknown track type: %s', track_type);
    end
end
