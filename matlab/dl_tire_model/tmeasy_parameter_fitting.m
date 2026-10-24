%% tmeasy_parameter_fitting.m
% 実データに基づくTMeasyパラメータのフィッティング

clear; clc;

fprintf('=================================================================\n');
fprintf(' TMeasy Parameter Fitting to Real Data\n');
fprintf('=================================================================\n\n');

%% ステップ1: 実データの読み込みと正規化
fprintf('Step 1: Loading and Analyzing Real Data\n');

data_path = '../../data/tiredata/';
corn_file = [data_path 'Cornering-Hoosier18.0×6.0-10-R20.csv'];

opts = detectImportOptions(corn_file);
opts.DataLines = [3, Inf];
opts.VariableNames = {'ET','V','N','SA','IA','RL','RE','P','FX','FY','FZ','MX','MZ','NFX','NFY','RST','TSTI','TSTC','TSTO','AmbTmp','SR','CP'};
corn_data = readtable(corn_file, opts);

% Fz低荷重データ抽出
Fz_actual = -corn_data.FZ;
Fz_min = 150;
Fz_max = 200;
idx_valid = Fz_actual > Fz_min & Fz_actual < Fz_max & abs(corn_data.SR) < 0.02;

corn_filtered = corn_data(idx_valid, :);
fprintf('  Extracted %d samples at Fz=[%.0f, %.0f]N\n', height(corn_filtered), Fz_min, Fz_max);

% 正規化
Fz_ref = 1000;
Fy_normalized = corn_filtered.FY .* (Fz_ref ./ (-corn_filtered.FZ));

fprintf('\nStep 2: Real Data Characteristics\n');

% SAごとの平均
SA_bins = -12:1:12;
Fy_data = zeros(size(SA_bins));
sample_count = zeros(size(SA_bins));

for i = 1:length(SA_bins)
    idx = abs(corn_filtered.SA - SA_bins(i)) < 0.5;
    if sum(idx) > 0
        Fy_data(i) = mean(Fy_normalized(idx));
        sample_count(i) = sum(idx);
    end
end

% 有効なデータポイント
valid_idx = sample_count > 0;
SA_valid = SA_bins(valid_idx);
Fy_valid = Fy_data(valid_idx);

fprintf('  Range of Fy: [%.0f, %.0f] N\n', min(Fy_valid), max(Fy_valid));
fprintf('  At SA=±3°: Fy ≈ %.0f N\n', abs(interp1(SA_valid, Fy_valid, 3, 'linear')));
fprintf('  At SA=±10°: Fy ≈ %.0f N\n', abs(interp1(SA_valid, Fy_valid, 10, 'linear')));

%% ステップ3: 実データから直接パラメータを推定
fprintf('\nStep 3: Direct Parameter Estimation from Real Data\n');

% 摩擦係数（最大値）
Fy_max_data = max(abs(Fy_valid));
mu_y_data = Fy_max_data / Fz_ref;
fprintf('  Maximum Fy: %.0f N\n', Fy_max_data);
fprintf('  Implied mu_y: %.3f\n', mu_y_data);

% コーナリングスティフネス（低スリップ角での傾き）
% SA=0～2°の範囲で線形フィッティング
idx_low = abs(SA_valid) <= 2;
if sum(idx_low) > 2
    SA_low = SA_valid(idx_low);
    Fy_low = Fy_valid(idx_low);
    
    % 対称性を考慮（-1,0,1,2と逆方向）
    SA_combined = [SA_low, -SA_low];
    Fy_combined = [Fy_low, -Fy_low];
    
    % 線形フィッティング: Fy = C_alpha * SA (deg単位)
    p_deg = polyfit(SA_combined, Fy_combined, 1);
    C_alpha_implicit = abs(p_deg(1));  % N/deg
    
    % rad単位に変換
    C_alpha_data = C_alpha_implicit * 180 / pi;
    
    fprintf('  Linear stiffness (SA=0-2°): %.0f N/deg\n', C_alpha_implicit);
    fprintf('  C_alpha: %.0f N/rad\n', C_alpha_data);
end

%% ステップ4: TMeasyモデルの調整
fprintf('\nStep 4: Adjusted TMeasy Parameters\n');

% 実データに合わせたパラメータ
C_alpha_fit = C_alpha_data;  % 推定値
C_kappa_fit = 80000;  % 縦方向はデータがないので仮定
mu_x_fit = 1.5;
mu_y_fit = mu_y_data;

fprintf('  C_alpha = %.0f N/rad\n', C_alpha_fit);
fprintf('  C_kappa = %.0f N\n', C_kappa_fit);
fprintf('  mu_x = %.2f\n', mu_x_fit);
fprintf('  mu_y = %.2f\n', mu_y_fit);

%% ステップ5: 複数のモデル比較
fprintf('\nStep 5: Model Comparison\n\n');

% パラメータセット
param_sets = struct();
param_sets(1).name = '実データから推定';
param_sets(1).C_alpha = C_alpha_fit;
param_sets(1).C_kappa = C_kappa_fit;
param_sets(1).mu_x = mu_x_fit;
param_sets(1).mu_y = mu_y_fit;

param_sets(2).name = '前回提案（保守的）';
param_sets(2).C_alpha = 45000;
param_sets(2).C_kappa = 80000;
param_sets(2).mu_x = 1.5;
param_sets(2).mu_y = 1.5;

param_sets(3).name = 'フォーミュラタイヤ標準';
param_sets(3).C_alpha = 50000;
param_sets(3).C_kappa = 100000;
param_sets(3).mu_x = 1.6;
param_sets(3).mu_y = 1.6;

% 全テストケースでの比較
test_cases = [-10, -5, 0, 5, 10];

fprintf('%-30s | ', 'Model');
for SA = test_cases
    fprintf('SA=%+3.0f°  | ', SA);
end
fprintf('\n');
fprintf('%s\n', repmat('-', 1, 30 + (length(test_cases)-1) * 12));

% 実データ
fprintf('%-30s | ', '実データ');
for SA = test_cases
    Fy_real = interp1(SA_valid, Fy_valid, SA, 'linear', NaN);
    if isnan(Fy_real)
        fprintf('%9s | ', 'N/A');
    else
        fprintf('%8.0f | ', Fy_real);
    end
end
fprintf('\n');

% 各モデル
for k = 1:length(param_sets)
    fprintf('%-30s | ', param_sets(k).name);
    for SA = test_cases
        [~, Fy_model] = tmeasy_force_model(0, SA, Fz_ref, param_sets(k));
        fprintf('%8.0f | ', Fy_model);
    end
    fprintf('\n');
end
fprintf('\n');

%% ステップ6: 誤差評価
fprintf('Step 6: Error Analysis\n\n');

for k = 1:length(param_sets)
    % SA全範囲での誤差
    Fy_model_all = zeros(size(SA_valid));
    for i = 1:length(SA_valid)
        [~, Fy_model_all(i)] = tmeasy_force_model(0, SA_valid(i), Fz_ref, param_sets(k));
    end
    
    % 誤差指標
    error_abs = abs(Fy_model_all - Fy_valid');
    error_rel = error_abs ./ abs(Fy_valid');
    error_rel(abs(Fy_valid') < 100) = NaN;  % ゼロ近辺を除外
    
    mae = mean(error_abs);
    mape = mean(error_rel, 'omitnan');
    rmse = sqrt(mean(error_abs.^2));
    
    fprintf('%s:\n', param_sets(k).name);
    fprintf('  MAE:  %.0f N\n', mae);
    fprintf('  MAPE: %.1f %%\n', mape * 100);
    fprintf('  RMSE: %.0f N\n', rmse);
    fprintf('\n');
end

%% ステップ7: グラフ描画
figure('Position', [100, 100, 1400, 900]);

% メインプロット：実データとモデル比較
subplot(2, 2, 1);
SA_plot = -12:0.5:12;

% 実データ
errorbar(SA_valid, Fy_valid, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', ...
         'LineWidth', 2, 'CapSize', 3);
hold on;

% 各モデル
colors = {'b', 'g', 'm'};
for k = 1:length(param_sets)
    Fy_plot = zeros(size(SA_plot));
    for i = 1:length(SA_plot)
        [~, Fy_plot(i)] = tmeasy_force_model(0, SA_plot(i), Fz_ref, param_sets(k));
    end
    plot(SA_plot, Fy_plot, [colors{k}, '-'], 'LineWidth', 2, 'DisplayName', param_sets(k).name);
end

xlabel('Slip Angle [deg]', 'FontSize', 11);
ylabel('Fy [N]', 'FontSize', 11);
title('Lateral Force Comparison (Fz=1000N)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Real Data', param_sets.name, 'Location', 'best', 'FontSize', 10);
grid on;
axis([-12, 12, -3500, 3500]);

% Fy/Fz（摩擦係数）プロット
subplot(2, 2, 2);
mu_y_real = Fy_valid / Fz_ref;

semilogy(SA_valid, abs(mu_y_real), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 2);
hold on;

for k = 1:length(param_sets)
    mu_y_model = ones(size(SA_plot)) * param_sets(k).mu_y;
    plot(SA_plot, mu_y_model, [colors{k}, '-'], 'LineWidth', 2, 'DisplayName', param_sets(k).name);
end

xlabel('Slip Angle [deg]', 'FontSize', 11);
ylabel('μy [-]', 'FontSize', 11);
title('Friction Coefficient (log scale)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Real Data', param_sets.name, 'Location', 'best', 'FontSize', 10);
grid on;
axis([-12, 12, 0.5, 5]);

% SA=5°でのFz依存性
subplot(2, 2, 3);
Fz_sweep = 300:100:2000;
Fy_sweep_data = [];
Fy_sweep_model = zeros(length(param_sets), length(Fz_sweep));

% 実データからの抽出（低荷重）
for i = 1:length(Fz_sweep)
    Fz = Fz_sweep(i);
    idx = abs(-corn_filtered.FZ - Fz) < 50 & abs(corn_filtered.SA - 5) < 1;
    if sum(idx) > 0
        Fy_sweep_data(i) = mean(corn_filtered.FY(idx));
    else
        Fy_sweep_data(i) = NaN;
    end
end

plot(Fz_sweep, Fy_sweep_data, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 2);
hold on;

for k = 1:length(param_sets)
    for i = 1:length(Fz_sweep)
        [~, Fy_sweep_model(k, i)] = tmeasy_force_model(0, 5, Fz_sweep(i), param_sets(k));
    end
    plot(Fz_sweep, Fy_sweep_model(k, :), [colors{k}, '-'], 'LineWidth', 2, 'DisplayName', param_sets(k).name);
end

xlabel('Vertical Load Fz [N]', 'FontSize', 11);
ylabel('Fy(SA=5°) [N]', 'FontSize', 11);
title('Load Dependency (SA=5°)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Real Data', param_sets.name, 'Location', 'best', 'FontSize', 10);
grid on;

% 推奨パラメータの摩擦円
subplot(2, 2, 4);
theta = 0:0.01:2*pi;
for k = 1:length(param_sets)
    Fx_ellipse = param_sets(k).mu_x * Fz_ref * cos(theta);
    Fy_ellipse = param_sets(k).mu_y * Fz_ref * sin(theta);
    plot(Fx_ellipse, Fy_ellipse, colors{k}, 'LineWidth', 2.5, 'DisplayName', param_sets(k).name);
    hold on;
end

xlabel('Fx [N]', 'FontSize', 11);
ylabel('Fy [N]', 'FontSize', 11);
title('Friction Ellipse (Fz=1000N)', 'FontSize', 12, 'FontWeight', 'bold');
legend(param_sets.name, 'Location', 'best', 'FontSize', 10);
axis equal;
grid on;
axis([-1500, 1500, -2500, 2500]);

saveas(gcf, '../../output/tire_params/tmeasy_fitting_comparison.png');
fprintf('\nFigure saved to output/tire_params/tmeasy_fitting_comparison.png\n');

%% 最終推奨
fprintf('\n=================================================================\n');
fprintf(' FINAL RECOMMENDATION\n');
fprintf('=================================================================\n\n');

fprintf('Based on real tire data analysis:\n\n');
fprintf('OPTION 1 - データ駆動アプローチ（推奨）:\n');
fprintf('  C_alpha = %.0f N/rad\n', C_alpha_fit);
fprintf('  C_kappa = 80000 N\n');
fprintf('  mu_x    = 1.50\n');
fprintf('  mu_y    = %.2f\n', mu_y_fit);
fprintf('  Note: 高いmu_yで実データに最も近い\n\n');

fprintf('OPTION 2 - 保守的アプローチ（安定性重視）:\n');
fprintf('  C_alpha = 45000 N/rad\n');
fprintf('  C_kappa = 80000 N\n');
fprintf('  mu_x    = 1.50\n');
fprintf('  mu_y    = 1.50\n');
fprintf('  Note: 低いmu_yで安全性を確保\n\n');

fprintf('推奨: OPTION 1でシミュレーション実行\n');
fprintf('理由: 実タイヤデータに基づいており、より正確な動力学予測が可能\n');
fprintf('=================================================================\n');

%% Local Function
function [Fx, Fy] = tmeasy_force_model(SR, SA, Fz, params)
    % TMeasy Force Model with given parameters
    
    alpha = SA * pi / 180;
    kappa = SR;
    
    Fx_max = params.mu_x * Fz;
    Fy_max = params.mu_y * Fz;
    
    Fx_lin = params.C_kappa * kappa;
    Fy_lin = -params.C_alpha * alpha;
    
    F_lin_mag = sqrt(Fx_lin^2 + Fy_lin^2);
    F_max = sqrt(Fx_max^2 + Fy_max^2);
    
    if F_lin_mag > 1e-6
        ratio = F_lin_mag / F_max;
        
        k = 1.8;
        if ratio < 3
            scale = tanh(ratio * k) / tanh(k);
        else
            scale = 1.0;
        end
        
        Fx = Fx_lin / F_lin_mag * F_max * scale;
        Fy = Fy_lin / F_lin_mag * F_max * scale;
        
        Fx = max(min(Fx, Fx_max), -Fx_max);
        Fy = max(min(Fy, Fy_max), -Fy_max);
    else
        Fx = 0;
        Fy = 0;
    end
end
