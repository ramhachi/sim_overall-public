%% tmeasy_realistic_fitting.m
% 現実的な荷重範囲（500-800N）での TMeasy パラメータフィッティング
% 低荷重（200N）データは外挿誤差が大きいため除外

clear; clc;

fprintf('=================================================================\n');
fprintf(' TMeasy Realistic Parameter Fitting (Fz=500-800N)\n');
fprintf('=================================================================\n\n');

%% ステップ1: データ読み込み
fprintf('Step 1: Loading Cornering Data\n');

data_path = '../../data/tiredata/';
corn_file = [data_path 'Cornering-Hoosier18.0×6.0-10-R20.csv'];

opts = detectImportOptions(corn_file);
opts.DataLines = [3, Inf];
opts.VariableNames = {'ET','V','N','SA','IA','RL','RE','P','FX','FY','FZ','MX','MZ','NFX','NFY','RST','TSTI','TSTC','TSTO','AmbTmp','SR','CP'};
corn_data = readtable(corn_file, opts);

% 荷重分布確認
Fz_all = -corn_data.FZ;
fprintf('  Total data statistics:\n');
fprintf('    Fz range: [%.0f, %.0f] N\n', min(Fz_all), max(Fz_all));
fprintf('    Fz mean: %.0f N\n', mean(Fz_all));
fprintf('    Fz median: %.0f N\n', median(Fz_all));

%% ステップ2: 現実的な荷重範囲でのデータ抽出
fprintf('\nStep 2: Extracting Realistic Load Range Data\n');

% 複数の荷重レベルで分析
Fz_ranges = [
    500, 600;   % 低荷重
    600, 700;   % 中荷重
    700, 800;   % 高荷重
];

results = struct();

for range_idx = 1:size(Fz_ranges, 1)
    Fz_min = Fz_ranges(range_idx, 1);
    Fz_max = Fz_ranges(range_idx, 2);
    Fz_ref = mean([Fz_min, Fz_max]);  % 基準荷重
    
    fprintf('\n--- Fz Range: [%.0f, %.0f]N (Ref: %.0f N) ---\n', Fz_min, Fz_max, Fz_ref);
    
    % データ抽出（SR≈0, 荷重範囲内）
    idx_valid = Fz_all > Fz_min & Fz_all < Fz_max & abs(corn_data.SR) < 0.02;
    corn_subset = corn_data(idx_valid, :);
    
    fprintf('  Samples: %d\n', height(corn_subset));
    
    % 基準荷重に正規化
    Fy_normalized = corn_subset.FY .* (Fz_ref ./ (-corn_subset.FZ));
    
    % SAごとの平均
    SA_bins = -12:1:12;
    Fy_mean = zeros(size(SA_bins));
    sample_count = zeros(size(SA_bins));
    
    for i = 1:length(SA_bins)
        idx = abs(corn_subset.SA - SA_bins(i)) < 0.5;
        if sum(idx) > 0
            Fy_mean(i) = mean(Fy_normalized(idx));
            sample_count(i) = sum(idx);
        end
    end
    
    % 有効データポイント
    valid_idx = sample_count > 10;  % 最低10サンプル
    SA_valid = SA_bins(valid_idx);
    Fy_valid = Fy_mean(valid_idx);
    
    % パラメータ推定
    % 1. コーナリングスティフネス（SA=0-2°の線形領域）
    idx_linear = abs(SA_valid) <= 2;
    if sum(idx_linear) >= 3
        SA_lin = SA_valid(idx_linear);
        Fy_lin = Fy_valid(idx_linear);
        p = polyfit(SA_lin, Fy_lin, 1);
        C_alpha_deg = abs(p(1));
        C_alpha_rad = C_alpha_deg * 180 / pi;
        
        fprintf('  Linear region (SA=0-2°):\n');
        fprintf('    Slope: %.0f N/deg\n', C_alpha_deg);
        fprintf('    C_alpha: %.0f N/rad\n', C_alpha_rad);
    else
        C_alpha_rad = NaN;
    end
    
    % 2. 最大摩擦係数（SA > 8°の飽和領域）
    idx_saturated = abs(SA_valid) > 8;
    if sum(idx_saturated) >= 3
        Fy_sat = Fy_valid(idx_saturated);
        Fy_sat_mean = mean(abs(Fy_sat));
        mu_y = Fy_sat_mean / Fz_ref;
        
        fprintf('  Saturated region (|SA|>8°):\n');
        fprintf('    Mean |Fy|: %.0f N\n', Fy_sat_mean);
        fprintf('    mu_y: %.3f\n', mu_y);
    else
        mu_y = NaN;
    end
    
    % 結果保存
    results(range_idx).Fz_ref = Fz_ref;
    results(range_idx).C_alpha = C_alpha_rad;
    results(range_idx).mu_y = mu_y;
    results(range_idx).SA_valid = SA_valid;
    results(range_idx).Fy_valid = Fy_valid;
end

%% ステップ3: 荷重依存性の分析
fprintf('\n=================================================================\n');
fprintf('Step 3: Load Dependency Analysis\n');
fprintf('=================================================================\n\n');

fprintf('%-15s | %-15s | %-15s\n', 'Fz Range [N]', 'C_alpha [N/rad]', 'mu_y [-]');
fprintf('%s\n', repmat('-', 1, 50));

C_alpha_all = [];
mu_y_all = [];
Fz_ref_all = [];

for i = 1:length(results)
    fprintf('%-15s | %-15.0f | %-15.3f\n', ...
            sprintf('[%.0f-%.0f]', Fz_ranges(i,1), Fz_ranges(i,2)), ...
            results(i).C_alpha, results(i).mu_y);
    
    if ~isnan(results(i).C_alpha)
        C_alpha_all = [C_alpha_all; results(i).C_alpha];
        Fz_ref_all = [Fz_ref_all; results(i).Fz_ref];
    end
    if ~isnan(results(i).mu_y)
        mu_y_all = [mu_y_all; results(i).mu_y];
    end
end

fprintf('\n');

% 荷重依存性の傾向
if length(C_alpha_all) >= 2
    % C_alphaは荷重によって変化する（通常は√Fzに比例）
    fprintf('C_alpha tendency:\n');
    fprintf('  Mean: %.0f N/rad\n', mean(C_alpha_all));
    fprintf('  Std: %.0f N/rad\n', std(C_alpha_all));
    fprintf('  Range: [%.0f, %.0f] N/rad\n', min(C_alpha_all), max(C_alpha_all));
end

if length(mu_y_all) >= 2
    fprintf('\nmu_y tendency:\n');
    fprintf('  Mean: %.3f\n', mean(mu_y_all));
    fprintf('  Std: %.3f\n', std(mu_y_all));
    fprintf('  Range: [%.3f, %.3f]\n', min(mu_y_all), max(mu_y_all));
end

%% ステップ4: 推奨パラメータの決定
fprintf('\n=================================================================\n');
fprintf('Step 4: Recommended Parameters\n');
fprintf('=================================================================\n\n');

% 中荷重（600-700N）を基準とする
C_alpha_rec = results(2).C_alpha;
mu_y_rec = mean(mu_y_all);  % 全範囲の平均
C_kappa_rec = C_alpha_rec * 1.5;  % 縦方向は横方向の約1.5倍が一般的
mu_x_rec = mu_y_rec * 0.9;  % 縦方向は横方向より若干低い

fprintf('Based on Fz=500-800N range:\n\n');
fprintf('  C_alpha = %.0f N/rad\n', C_alpha_rec);
fprintf('  C_kappa = %.0f N (estimated)\n', C_kappa_rec);
fprintf('  mu_x    = %.2f (estimated)\n', mu_x_rec);
fprintf('  mu_y    = %.2f\n', mu_y_rec);
fprintf('\n');

% 保守的な値
C_alpha_conservative = floor(C_alpha_rec / 5000) * 5000;  % 5000の倍数に丸める
mu_y_conservative = floor(mu_y_rec * 10) / 10;

fprintf('Conservative (rounded down):\n\n');
fprintf('  C_alpha = %.0f N/rad\n', C_alpha_conservative);
fprintf('  C_kappa = %.0f N\n', floor(C_kappa_rec / 5000) * 5000);
fprintf('  mu_x    = %.1f\n', floor(mu_x_rec * 10) / 10);
fprintf('  mu_y    = %.1f\n', mu_y_conservative);
fprintf('\n');

%% ステップ5: モデル検証
fprintf('=================================================================\n');
fprintf('Step 5: Model Validation\n');
fprintf('=================================================================\n\n');

% 中荷重データでの検証
test_idx = 2;  % 600-700N range
SA_test = results(test_idx).SA_valid;
Fy_data = results(test_idx).Fy_valid;
Fz_test = results(test_idx).Fz_ref;

% TMeasyモデルでの計算
Fy_model_rec = zeros(size(SA_test));
Fy_model_cons = zeros(size(SA_test));

params_rec.C_alpha = C_alpha_rec;
params_rec.C_kappa = C_kappa_rec;
params_rec.mu_x = mu_x_rec;
params_rec.mu_y = mu_y_rec;

params_cons.C_alpha = C_alpha_conservative;
params_cons.C_kappa = floor(C_kappa_rec / 5000) * 5000;
params_cons.mu_x = floor(mu_x_rec * 10) / 10;
params_cons.mu_y = mu_y_conservative;

for i = 1:length(SA_test)
    [~, Fy_model_rec(i)] = tmeasy_force_calc(0, SA_test(i), Fz_test, params_rec);
    [~, Fy_model_cons(i)] = tmeasy_force_calc(0, SA_test(i), Fz_test, params_cons);
end

% 誤差計算（次元を揃える）
Fy_data_vec = Fy_data(:);  % 列ベクトルに変換
Fy_model_rec_vec = Fy_model_rec(:);  % 列ベクトルに変換
Fy_model_cons_vec = Fy_model_cons(:);  % 列ベクトルに変換

error_rec = abs(Fy_model_rec_vec - Fy_data_vec);
error_cons = abs(Fy_model_cons_vec - Fy_data_vec);

% ゼロ近辺を除外して相対誤差を計算
valid_for_mape = abs(Fy_data_vec) > 100;
mape_rec = mean(error_rec(valid_for_mape) ./ abs(Fy_data_vec(valid_for_mape))) * 100;
mape_cons = mean(error_cons(valid_for_mape) ./ abs(Fy_data_vec(valid_for_mape))) * 100;

fprintf('Validation at Fz=%.0fN:\n\n', Fz_test);
fprintf('Recommended parameters:\n');
fprintf('  MAE:  %.0f N\n', mean(error_rec));
fprintf('  MAPE: %.1f %%\n', mape_rec);
fprintf('  RMSE: %.0f N\n', sqrt(mean(error_rec.^2)));
fprintf('\n');

fprintf('Conservative parameters:\n');
fprintf('  MAE:  %.0f N\n', mean(error_cons));
fprintf('  MAPE: %.1f %%\n', mape_cons);
fprintf('  RMSE: %.0f N\n', sqrt(mean(error_cons.^2)));
fprintf('\n');

% 比較表
fprintf('Comparison at key slip angles:\n');
fprintf('%-10s | %-12s | %-12s | %-12s\n', 'SA [deg]', 'Real Data', 'Recommended', 'Conservative');
fprintf('%s\n', repmat('-', 1, 55));

test_SA = [-10, -5, 0, 5, 10];
for SA = test_SA
    Fy_real = interp1(SA_test, Fy_data, SA, 'linear', NaN);
    [~, Fy_rec] = tmeasy_force_calc(0, SA, Fz_test, params_rec);
    [~, Fy_cons] = tmeasy_force_calc(0, SA, Fz_test, params_cons);
    
    if ~isnan(Fy_real)
        fprintf('%-10.0f | %-12.0f | %-12.0f | %-12.0f\n', SA, Fy_real, Fy_rec, Fy_cons);
    end
end
fprintf('\n');

%% ステップ6: グラフ描画
figure('Position', [100, 100, 1400, 900]);

% Fy vs SA（全荷重範囲）
subplot(2, 2, 1);
colors_data = {'r', 'g', 'b'};
hold on;

for i = 1:length(results)
    SA = results(i).SA_valid;
    Fy = results(i).Fy_valid;
    plot(SA, Fy, [colors_data{i}, 'o'], 'MarkerSize', 6, 'MarkerFaceColor', colors_data{i}, ...
         'DisplayName', sprintf('Data Fz=%.0fN', results(i).Fz_ref));
end

% 推奨モデル
SA_plot = -12:0.5:12;
Fy_plot_rec = zeros(size(SA_plot));
Fy_plot_cons = zeros(size(SA_plot));

for i = 1:length(SA_plot)
    [~, Fy_plot_rec(i)] = tmeasy_force_calc(0, SA_plot(i), Fz_test, params_rec);
    [~, Fy_plot_cons(i)] = tmeasy_force_calc(0, SA_plot(i), Fz_test, params_cons);
end

plot(SA_plot, Fy_plot_rec, 'k-', 'LineWidth', 2.5, 'DisplayName', 'Recommended');
plot(SA_plot, Fy_plot_cons, 'k--', 'LineWidth', 2, 'DisplayName', 'Conservative');

xlabel('Slip Angle [deg]', 'FontSize', 12);
ylabel('Fy [N]', 'FontSize', 12);
title(sprintf('Lateral Force (Fz=%.0fN)', Fz_test), 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;

% 荷重依存性
subplot(2, 2, 2);
Fz_sweep = 400:50:1000;
Fy_sweep_rec = zeros(size(Fz_sweep));
Fy_sweep_cons = zeros(size(Fz_sweep));

for i = 1:length(Fz_sweep)
    [~, Fy_sweep_rec(i)] = tmeasy_force_calc(0, 5, Fz_sweep(i), params_rec);
    [~, Fy_sweep_cons(i)] = tmeasy_force_calc(0, 5, Fz_sweep(i), params_cons);
end

% 実データポイント（SA=5°付近）
for i = 1:length(results)
    SA_data = results(i).SA_valid;
    Fy_data_pt = results(i).Fy_valid;
    idx_5deg = find(abs(SA_data - 5) < 0.5, 1);
    if ~isempty(idx_5deg)
        plot(results(i).Fz_ref, Fy_data_pt(idx_5deg), [colors_data{i}, 'o'], ...
             'MarkerSize', 10, 'MarkerFaceColor', colors_data{i}, 'LineWidth', 2);
        hold on;
    end
end

plot(Fz_sweep, Fy_sweep_rec, 'k-', 'LineWidth', 2.5, 'DisplayName', 'Recommended');
plot(Fz_sweep, Fy_sweep_cons, 'k--', 'LineWidth', 2, 'DisplayName', 'Conservative');

xlabel('Vertical Load Fz [N]', 'FontSize', 12);
ylabel('Fy (SA=5°) [N]', 'FontSize', 12);
title('Load Dependency at SA=5°', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;

% 摩擦円
subplot(2, 2, 3);
theta = 0:0.01:2*pi;

% 推奨
Fx_rec = params_rec.mu_x * Fz_test * cos(theta);
Fy_rec = params_rec.mu_y * Fz_test * sin(theta);
plot(Fx_rec, Fy_rec, 'k-', 'LineWidth', 2.5, 'DisplayName', 'Recommended');
hold on;

% 保守的
Fx_cons = params_cons.mu_x * Fz_test * cos(theta);
Fy_cons = params_cons.mu_y * Fz_test * sin(theta);
plot(Fx_cons, Fy_cons, 'k--', 'LineWidth', 2, 'DisplayName', 'Conservative');

xlabel('Fx [N]', 'FontSize', 12);
ylabel('Fy [N]', 'FontSize', 12);
title(sprintf('Friction Ellipse (Fz=%.0fN)', Fz_test), 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
axis equal;
grid on;

% 誤差プロット
subplot(2, 2, 4);

% スカラー値として明示的に扱う
mae_rec = mean(error_rec);
mae_cons = mean(error_cons);
rmse_rec = sqrt(mean(error_rec.^2));
rmse_cons = sqrt(mean(error_cons.^2));

bar_data = [mae_rec, mae_cons; ...
            mape_rec, mape_cons; ...
            rmse_rec, rmse_cons];

bar(bar_data);
set(gca, 'XTickLabel', {'MAE [N]', 'MAPE [%]', 'RMSE [N]'});
ylabel('Error', 'FontSize', 12);
title('Model Accuracy Comparison', 'FontSize', 13, 'FontWeight', 'bold');
legend('Recommended', 'Conservative', 'Location', 'best', 'FontSize', 10);
grid on;

saveas(gcf, '../../output/tire_params/tmeasy_realistic_fitting.png');
fprintf('Figure saved to output/tire_params/tmeasy_realistic_fitting.png\n');

%% 最終結論
fprintf('\n=================================================================\n');
fprintf(' FINAL RECOMMENDATION FOR 14DOF SIMULATION\n');
fprintf('=================================================================\n\n');

fprintf('★ RECOMMENDED PARAMETERS (Data-driven, Fz=500-800N):\n\n');
fprintf('  params.tire.C_alpha = %.0f;  %% N/rad\n', C_alpha_rec);
fprintf('  params.tire.C_kappa = %.0f;  %% N\n', C_kappa_rec);
fprintf('  params.tire.mu_x    = %.2f;\n', mu_x_rec);
fprintf('  params.tire.mu_y    = %.2f;\n', mu_y_rec);
fprintf('\n');
fprintf('  Accuracy: MAE=%.0fN, MAPE=%.1f%%\n', mean(error_rec), mape_rec);
fprintf('\n');

fprintf('◆ CONSERVATIVE PARAMETERS (Safety margin):\n\n');
fprintf('  params.tire.C_alpha = %.0f;  %% N/rad\n', C_alpha_conservative);
fprintf('  params.tire.C_kappa = %.0f;  %% N\n', floor(C_kappa_rec / 5000) * 5000);
fprintf('  params.tire.mu_x    = %.1f;\n', floor(mu_x_rec * 10) / 10);
fprintf('  params.tire.mu_y    = %.1f;\n', mu_y_conservative);
fprintf('\n');
fprintf('  Accuracy: MAE=%.0fN, MAPE=%.1f%%\n', mean(error_cons), mape_cons);
fprintf('\n');

fprintf('推奨: RECOMMENDED パラメータで14DOFシミュレーションを実行\n');
fprintf('理由: 現実的な荷重範囲（500-800N）での実測に基づく\n');
fprintf('=================================================================\n');

%% Local Function
function [Fx, Fy] = tmeasy_force_calc(SR, SA, Fz, params)
    % TMeasy Force Calculation
    
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
