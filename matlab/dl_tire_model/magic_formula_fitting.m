%% magic_formula_fitting.m
% Pacejka Magic Formula (MF 5.2) タイヤモデル - 完全版フィッティング
%
% 基本方程式:
%   y = D·sin(C·arctan(B·x - E·(B·x - arctan(B·x)))) + SV
%
% 4段階フィッティング:
%   Stage 1: 純横力 Fy0(α, Fz, γ)  ← Cornering データ (|SR|<0.05)
%   Stage 2: 純縦力 Fx0(κ, Fz, γ)  ← DriveBrake データ (|SA|<1°)
%   Stage 3: 複合スリップ Gxα, Gyκ ← DriveBrake 全データ
%   Stage 4: セルフアライニングトルク Mz ← Cornering データ
%
% データ入力: loadTireData() 使用 (clean_and_retrain.m と同一方式)
% 最適化: lsqnonlin (trust-region-reflective, 境界付き)

%% ============================================================
%% ユーザ設定
%% ============================================================

if ~exist('run_from_launcher', 'var')
    clear; clc; close all;
end

if ~exist('tire_model', 'var')
    tire_model = 'Hoosier_18p0X6p0-10_R20';
end

if ~exist('data_dir', 'var')
    % Default relative path from this script location
    % If running from EasyStart, pwd is likely project root
    if exist('data/tiredata/csv', 'dir')
        data_dir = 'data/tiredata/csv';
    else
        % Fallback for relative path if running from this folder
        data_dir = '../../data/tiredata/csv';
    end
end

% 公称荷重 [N] (正規化基準)
Fz0 = -890;

% タイヤ無負荷半径 [m] (Hoosier 18.0x6.0-10: 直径18")
R0 = 18.0 / 2 * 0.0254;  % = 0.2286 m

% データ選別
db_exclude_runs = {'Run068', 'Run071'};  % SA sweep Runs (SR固定) → 除外
cr_sr_threshold = 0.05;   % Cornering: |SR| < 0.05 のみ
db_sa_threshold_pure = 1.0;  % DriveBrake純縦力: |SA| < 1° のみ

% 最適化オプション
max_subsample = 20000;  % フィッティング用サブサンプル数
rng(42);

%% ============================================================
%% Step 1: データ読み込み (clean_and_retrain.m 方式)
%% ============================================================
fprintf('Step 1: データ読み込み\n\n');

% --- Cornering ---
cr_pattern = fullfile(data_dir, ['Cr_' tire_model '*.csv']);
cr_found = dir(cr_pattern);
cr_paths = cellfun(@(f) fullfile(data_dir, f), sort({cr_found.name}'), 'UniformOutput', false);
cr_raw = loadTireData(cr_paths);
fprintf('  Cornering 生データ: %d サンプル\n', height(cr_raw));

% --- DriveBrake (Run選別) ---
db_pattern = fullfile(data_dir, ['Db_' tire_model '*.csv']);
db_found = dir(db_pattern);
db_all_names = sort({db_found.name}');
db_keep_mask = true(size(db_all_names));
for i = 1:length(db_all_names)
    for j = 1:length(db_exclude_runs)
        if contains(db_all_names{i}, db_exclude_runs{j})
            db_keep_mask(i) = false;
            fprintf('  除外: %s (SA sweep, SR固定)\n', db_all_names{i});
        end
    end
end
db_names = db_all_names(db_keep_mask);
db_paths = cellfun(@(f) fullfile(data_dir, f), db_names, 'UniformOutput', false);
db_raw = loadTireData(db_paths);
fprintf('  DriveBrake 生データ: %d サンプル (%d/%d ファイル)\n\n', ...
    height(db_raw), length(db_names), length(db_all_names));

%% ============================================================
%% Step 2: データ準備
%% ============================================================
fprintf('Step 2: データ準備\n\n');

% --- 純横力用: Cornering (|SR|<0.05) ---
cr_pure = cr_raw(abs(cr_raw.SR) < cr_sr_threshold, :);
fprintf('  純横力用 (Cornering |SR|<%.2f): %d サンプル\n', cr_sr_threshold, height(cr_pure));

% --- 純縦力用: DriveBrake (|SA|<1°) ---
db_pure = db_raw(abs(db_raw.SA) < db_sa_threshold_pure, :);
fprintf('  純縦力用 (DriveBrake |SA|<%.1f°): %d サンプル\n', db_sa_threshold_pure, height(db_pure));

% --- 複合スリップ用: DriveBrake (|SA|>1°) ---
db_comb = db_raw(abs(db_raw.SA) >= db_sa_threshold_pure, :);
fprintf('  複合スリップ用 (DriveBrake |SA|>=%.1f°): %d サンプル\n\n', ...
    db_sa_threshold_pure, height(db_comb));

% サブサンプリング (計算効率化)
cr_fit = subsample(cr_pure, max_subsample);
db_fit = subsample(db_pure, max_subsample);
db_comb_fit = subsample(db_comb, max_subsample);

fprintf('  フィッティング用サブサンプル:\n');
fprintf('    Cornering: %d, DriveBrake純: %d, DriveBrake複合: %d\n\n', ...
    height(cr_fit), height(db_fit), height(db_comb_fit));

%% ============================================================
%% Stage 1: 純横力 Fy0 フィッティング
%% ============================================================
fprintf('=== Stage 1: 純横力 Fy0 フィッティング ===\n\n');

% データ抽出
alpha_fy = deg2rad(cr_fit.SA);  % [rad]
Fz_fy = cr_fit.Fz;             % [N]
gamma_fy = deg2rad(cr_fit.IA); % [rad]
Fy_meas = cr_fit.Fy;           % [N]

% パラメータ定義:
%  [pCy1, pDy1, pDy2, pDy3, pKy1, pKy2, pKy4,
%   pEy1, pEy2, pEy3, pEy4, pHy1, pHy2, pVy1, pVy2]
%
% pCy1: 形状係数 (1.0-2.0)
% pDy1,2: ピーク摩擦係数 (Fz依存)
% pDy3: キャンバ影響
% pKy1,2,4: コーナリング剛性 (Fz依存)
% pEy1-4: 曲率 (Fz依存, 非対称)
% pHy1,2: 水平シフト
% pVy1,2: 垂直シフト

fy_names = {'pCy1','pDy1','pDy2','pDy3','pKy1','pKy2','pKy4',...
             'pEy1','pEy2','pEy3','pEy4','pHy1','pHy2','pVy1','pVy2'};

% 初期推定値をデータから計算
mu_y_est = max(abs(Fy_meas)) / median(Fz_fy);
Kya_est = estimateStiffness(cr_fit.SA, cr_fit.Fy, cr_fit.Fz, Fz0, 'lateral');

fy_x0 = [1.3, mu_y_est, -0.1, 0, ...
          Kya_est, 2.0, 2.0, ...
          -0.5, 0, 0, 0, ...
          0, 0, 0, 0];

fy_lb = [1.0, 0.5, -2.0, -0.5, ...
         -60, 0.5, 0.5, ...
         -5, -3, -3, -3, ...
         -0.1, -0.1, -0.5, -0.5];

fy_ub = [2.0, 3.5, 1.0, 0.5, ...
         60, 10, 4.0, ...
         1.0, 3, 3, 3, ...
         0.1, 0.1, 0.5, 0.5];

% 目的関数
obj_fy = @(p) calc_Fy0(p, alpha_fy, Fz_fy, gamma_fy, Fz0) - Fy_meas;

% 最適化
opts = optimoptions('lsqnonlin', ...
    'Algorithm', 'trust-region-reflective', ...
    'MaxIterations', 50000, ...
    'MaxFunctionEvaluations', 500000, ...
    'FunctionTolerance', 1e-10, ...
    'StepTolerance', 1e-10, ...
    'Display', 'iter-detailed');

fprintf('  最適化開始 (N=%d, params=%d)...\n', length(Fy_meas), length(fy_x0));
tic;
[fy_params, fy_resnorm, fy_residuals, ~, ~, ~, fy_jacobian] = ...
    lsqnonlin(obj_fy, fy_x0, fy_lb, fy_ub, opts);
fy_time = toc;

% 全データで評価
Fy_pred_all = calc_Fy0(fy_params, deg2rad(cr_pure.SA), cr_pure.Fz, deg2rad(cr_pure.IA), Fz0);
fy_rmse = sqrt(mean((Fy_pred_all - cr_pure.Fy).^2));
fy_r2 = 1 - sum((cr_pure.Fy - Fy_pred_all).^2) / sum((cr_pure.Fy - mean(cr_pure.Fy)).^2);

fprintf('\n  ✓ Fy0 完了 (%.1f sec)\n', fy_time);
fprintf('    RMSE = %.1f N, R² = %.4f\n', fy_rmse, fy_r2);
printParams('Fy0', fy_names, fy_params);

%% ============================================================
%% Stage 2: 純縦力 Fx0 フィッティング (二段階 + マルチスタート)
%% ============================================================
fprintf('\n=== Stage 2: 純縦力 Fx0 フィッティング ===\n\n');

% パラメータ名
fx_names = {'pCx1','pDx1','pDx2','pDx3','pKx1','pKx2','pKx3',...
             'pEx1','pEx2','pEx3','pEx4','pHx1','pHx2','pVx1','pVx2'};

% データ抽出
kappa_fx = db_fit.SR;
Fz_fx = db_fit.Fz;
gamma_fx = deg2rad(db_fit.IA);
Fx_meas = db_fit.Fx;

fprintf('  データ: %d点, SR=[%.3f,%.3f], Fx=[%.0f,%.0f], Fz=[%.0f,%.0f]\n', ...
    length(Fx_meas), min(kappa_fx), max(kappa_fx), ...
    min(Fx_meas), max(Fx_meas), min(Fz_fx), max(Fz_fx));

% ── Phase 1: 参照荷重 Fz≈Fz0 での簡易4パラメータ MF ──────────
fprintf('\n  Phase 1: 参照荷重 Fz≈%.0fN での簡易フィット (4 params)\n', Fz0);

ref_mask = abs(db_fit.Fz - Fz0) < 0.3 * Fz0;
if sum(ref_mask) < 300
    ref_mask = abs(db_fit.Fz - Fz0) < 0.5 * Fz0;
end
kappa_ref = db_fit.SR(ref_mask);
Fx_ref = db_fit.Fx(ref_mask);
Fz_ref_mean = mean(db_fit.Fz(ref_mask));
fprintf('    参照データ: %d点 (Fz=%.0f±%.0fN)\n', ...
    sum(ref_mask), Fz_ref_mean, std(db_fit.Fz(ref_mask)));

% データ駆動 初期推定
D_est = prctile(abs(Fx_ref), 97);
small_mask = abs(kappa_ref) < 0.03;
if sum(small_mask) > 30
    pf = polyfit(kappa_ref(small_mask), Fx_ref(small_mask), 1);
    slope_est = pf(1);
else
    slope_est = D_est * 10;  % fallback
end
B_est = abs(slope_est) / max(1.65 * D_est, 100);
fprintf('    推定: D=%.0fN, slope=%.0fN/SR, B=%.2f\n', D_est, slope_est, B_est);

% 簡易 MF (κのみ、Fz依存なし): Fx = D*sin(C*atan(Bκ - E*(Bκ - atan(Bκ))))
obj_ph1 = @(p) p(3).*sin(p(2).*atan(p(1).*kappa_ref - ...
    p(4).*(p(1).*kappa_ref - atan(p(1).*kappa_ref)))) - Fx_ref;

opts_ph1 = optimoptions('lsqnonlin', 'Algorithm','trust-region-reflective', ...
    'MaxIterations',50000, 'MaxFunctionEvaluations',500000, ...
    'FunctionTolerance',1e-12, 'StepTolerance',1e-12, 'Display','off');
lb_ph1 = [0.1, 0.8, 50, -5];
ub_ph1 = [200, 2.5, 8000, 1];

best_rn1 = inf; best_p1 = [B_est, 1.65, D_est, -0.3];
for trial = 1:10
    switch trial
        case 1, x0 = [B_est, 1.65, D_est, -0.3];
        case 2, x0 = [B_est*2, 1.3, D_est*0.8, 0];
        case 3, x0 = [B_est*0.5, 1.9, D_est*1.2, -1.5];
        case 4, x0 = [10, 1.5, D_est, -0.5];
        case 5, x0 = [30, 1.65, D_est*0.7, 0.5];
        otherwise, x0 = [B_est*(0.2+2*rand), 0.9+1.2*rand, ...
                         D_est*(0.4+1.2*rand), -3+4*rand];
    end
    x0 = max(min(x0, ub_ph1), lb_ph1);
    try
        [p1, rn1] = lsqnonlin(obj_ph1, x0, lb_ph1, ub_ph1, opts_ph1);
        if rn1 < best_rn1, best_rn1 = rn1; best_p1 = p1; end
    catch
        % skip
    end
end

B_ref = best_p1(1); C_ref = best_p1(2);
D_ref = best_p1(3); E_ref = best_p1(4);
mu_ref = D_ref / Fz_ref_mean;
fprintf('    Phase 1 結果: B=%.3f, C=%.3f, D=%.1fN (μ=%.3f), E=%.3f\n', ...
    B_ref, C_ref, D_ref, mu_ref, E_ref);
fprintf('    Phase 1 RMSE: %.1fN\n\n', sqrt(best_rn1/sum(ref_mask)));

% ── Phase 2: 全荷重 15パラメータ × マルチスタート ─────────────
fprintf('  Phase 2: 全荷重 15パラメータ × マルチスタート\n');

pKx1_init = B_ref * C_ref * mu_ref;  % Kxk = Fz*pKx1, B = pKx1/(C*mu)
fx_x0_base = [C_ref, mu_ref, -0.1, 0, ...
              pKx1_init, 0, 0, ...
              E_ref, 0, 0, 0, ...
              0, 0, 0, 0];

fx_lb = [0.8, 0.3, -3, -0.5, ...
         0.5, -15, -5, ...
         -5, -5, -5, -1, ...
         -0.005, -0.01, -0.005, -0.02];
fx_ub = [2.5, 5.0, 2, 0.5, ...
         100, 15, 5, ...
         1, 5, 5, 1, ...
         0.005, 0.01, 0.005, 0.02];

fx_x0_base = max(min(fx_x0_base, fx_ub), fx_lb);

obj_fx = @(p) calc_Fx0(p, kappa_fx, Fz_fx, gamma_fx, Fz0) - Fx_meas;

opts_fx = optimoptions('lsqnonlin', 'Algorithm','trust-region-reflective', ...
    'MaxIterations',50000, 'MaxFunctionEvaluations',500000, ...
    'FunctionTolerance',1e-12, 'StepTolerance',1e-12, ...
    'Display','iter-detailed');

fprintf('  マルチスタート最適化 (5回)...\n');
best_rn2 = inf; best_fx = fx_x0_base;
for trial = 1:5
    if trial == 1
        x0_15 = fx_x0_base;
    else
        x0_15 = fx_x0_base;
        x0_15(1) = C_ref * (0.7 + 0.6*rand);
        x0_15(2) = mu_ref * (0.7 + 0.6*rand);
        x0_15(5) = pKx1_init * (0.5 + rand);
        x0_15(8) = -2 + 3*rand;
    end
    x0_15 = max(min(x0_15, fx_ub), fx_lb);
    fprintf('\n  --- Trial %d/5 ---\n', trial);
    tic;
    try
        [fx_p, fx_rn] = lsqnonlin(obj_fx, x0_15, fx_lb, fx_ub, opts_fx);
        t = toc;
        fprintf('  Trial %d: resnorm=%.4e (%.1fs)\n', trial, fx_rn, t);
        if fx_rn < best_rn2, best_rn2 = fx_rn; best_fx = fx_p; end
    catch e
        fprintf('  Trial %d: FAILED - %s\n', trial, e.message);
    end
end
fx_params = best_fx;

% 全データで評価
Fx_pred_all = calc_Fx0(fx_params, db_pure.SR, db_pure.Fz, deg2rad(db_pure.IA), Fz0);
fx_rmse = sqrt(mean((Fx_pred_all - db_pure.Fx).^2));
fx_r2 = 1 - sum((db_pure.Fx - Fx_pred_all).^2) / sum((db_pure.Fx - mean(db_pure.Fx)).^2);

fprintf('\n  ✓ Fx0 完了 (best resnorm=%.4e)\n', best_rn2);
fprintf('    RMSE = %.1f N, R² = %.4f\n', fx_rmse, fx_r2);
printParams('Fx0', fx_names, fx_params);

%% ============================================================
%% Stage 3: 複合スリップ フィッティング
%% ============================================================
fprintf('\n=== Stage 3: 複合スリップ フィッティング ===\n\n');

% データ: DriveBrake |SA|>1° (複合スリップ条件)
kappa_c = db_comb_fit.SR;
alpha_c = deg2rad(db_comb_fit.SA);
Fz_c = db_comb_fit.Fz;
gamma_c = deg2rad(db_comb_fit.IA);

% 純スリップでの予測値
Fx0_pred = calc_Fx0(fx_params, kappa_c, Fz_c, gamma_c, Fz0);
Fy0_pred = calc_Fy0(fy_params, alpha_c, Fz_c, gamma_c, Fz0);

% --- Gxα フィッティング (Fx の SA による減少) ---
% パラメータ: [rBx1, rBx2, rCx1, rHx1, rEx1]
gxa_names = {'rBx1','rBx2','rCx1','rHx1','rEx1'};
gxa_x0 = [12, 8, 1.0, 0, 0];
gxa_lb = [1, 0.1, 0.5, -0.1, -5];
gxa_ub = [30, 25, 2.0, 0.1, 2];

Fx_comb_meas = db_comb_fit.Fx;
obj_gxa = @(p) calc_Gxa(p, alpha_c, kappa_c, Fz_c, Fz0) .* Fx0_pred - Fx_comb_meas;

opts_comb = optimoptions('lsqnonlin', ...
    'Algorithm', 'trust-region-reflective', ...
    'MaxIterations', 50000, ...
    'MaxFunctionEvaluations', 500000, ...
    'Display', 'iter');

fprintf('  Gxα 最適化 (N=%d)...\n', length(Fx_comb_meas));
tic;
[gxa_params, ~] = lsqnonlin(obj_gxa, gxa_x0, gxa_lb, gxa_ub, opts_comb);
gxa_time = toc;

% --- Gyκ フィッティング (Fy の SR による減少) ---
% パラメータ: [rBy1, rBy2, rBy3, rCy1, rHy1]
gyk_names = {'rBy1','rBy2','rBy3','rCy1','rHy1'};
gyk_x0 = [8, 5, 0, 1.0, 0];
gyk_lb = [1, 0.1, -0.2, 0.5, -0.1];
gyk_ub = [30, 25, 0.2, 2.0, 0.1];

Fy_comb_meas = db_comb_fit.Fy;
obj_gyk = @(p) calc_Gyk(p, alpha_c, kappa_c, Fz_c, Fz0) .* Fy0_pred - Fy_comb_meas;

fprintf('  Gyκ 最適化 (N=%d)...\n', length(Fy_comb_meas));
[gyk_params, ~] = lsqnonlin(obj_gyk, gyk_x0, gyk_lb, gyk_ub, opts_comb);
gyk_time = toc;

% 複合スリップ評価 (全データ)
Fx0_full = calc_Fx0(fx_params, db_raw.SR, db_raw.Fz, deg2rad(db_raw.IA), Fz0);
Fy0_full = calc_Fy0(fy_params, deg2rad(db_raw.SA), db_raw.Fz, deg2rad(db_raw.IA), Fz0);
Gxa_full = calc_Gxa(gxa_params, deg2rad(db_raw.SA), db_raw.SR, db_raw.Fz, Fz0);
Gyk_full = calc_Gyk(gyk_params, deg2rad(db_raw.SA), db_raw.SR, db_raw.Fz, Fz0);
Fx_comb_pred = Gxa_full .* Fx0_full;
Fy_comb_pred = Gyk_full .* Fy0_full;

fx_comb_rmse = sqrt(mean((Fx_comb_pred - db_raw.Fx).^2));
fy_comb_rmse = sqrt(mean((Fy_comb_pred - db_raw.Fy).^2));
fx_comb_r2 = 1 - sum((db_raw.Fx - Fx_comb_pred).^2) / sum((db_raw.Fx - mean(db_raw.Fx)).^2);
fy_comb_r2 = 1 - sum((db_raw.Fy - Fy_comb_pred).^2) / sum((db_raw.Fy - mean(db_raw.Fy)).^2);

fprintf('\n  ✓ 複合スリップ完了\n');
fprintf('    Fx(複合): RMSE=%.1fN, R²=%.4f\n', fx_comb_rmse, fx_comb_r2);
fprintf('    Fy(複合): RMSE=%.1fN, R²=%.4f\n', fy_comb_rmse, fy_comb_r2);
printParams('Gxα', gxa_names, gxa_params);
printParams('Gyκ', gyk_names, gyk_params);

%% ============================================================
%% Stage 4: セルフアライニングトルク Mz フィッティング
%% ============================================================
fprintf('\n=== Stage 4: Mz フィッティング ===\n\n');

% Mz = -trail * Fy + Mzr (ニューマチックトレイル + 残留トルク)
% trail = Dt*cos(Ct*atan(Bt*αt - Et*(Bt*αt - atan(Bt*αt))))*cos(α)
% Mzr = Dr*cos(atan(Br*αr))
%
% パラメータ: [qBz1, qBz2, qCz1, qDz1, qDz2, qEz1,
%              qHz1, qHz2, qBz9, qDz6, qDz8, qDz9]

mz_names = {'qBz1','qBz2','qCz1','qDz1','qDz2','qEz1',...
             'qHz1','qHz2','qBz9','qDz6','qDz8','qDz9'};

alpha_mz = deg2rad(cr_fit.SA);
Fz_mz = cr_fit.Fz;
gamma_mz = deg2rad(cr_fit.IA);
Mz_meas = cr_fit.Mz;

% Fy0 予測値 (トレイル計算に使用)
Fy0_mz = calc_Fy0(fy_params, alpha_mz, Fz_mz, gamma_mz, Fz0);

% Kyα (Fy0 パラメータから計算)
Kya_mz = calcKya(fy_params, Fz_mz, gamma_mz, Fz0);

% SHy, SVy (Fy0 パラメータから)
dfz_mz = (Fz_mz - Fz0) ./ Fz0;
SHy_mz = fy_params(12) + fy_params(13) * dfz_mz;
SVy_mz = Fz_mz .* (fy_params(14) + fy_params(15) * dfz_mz);

% --- 物理ベース初期推定 ---
% trail ≈ 20-40mm → qDz1 = trail/(R0) ≈ 0.09-0.17
% Mzr peak ≈ 5-15Nm → qDz6 ≈ Mzr_peak/(Fz0*R0) ≈ 0.025-0.075
fprintf('  R0=%.4f m (Mzパラメータ正規化用)\n', R0);

mz_x0_base = [10, 0, 1.3, 0.13, 0, -1, 0, 0, 8, -0.03, -0.01, 0];
mz_lb = [0.5, -15, 0.5, 0.01, -1.0, -10, -0.3, -0.3, 0.1, -0.5, -0.5, -0.5];
mz_ub = [40,   15, 2.5, 1.0,   1.0,   2,  0.3,  0.3, 50,  0.5,  0.5,  0.5];

obj_mz = @(p) calc_Mz(p, alpha_mz, Fz_mz, gamma_mz, Fy0_mz, Kya_mz, SHy_mz, SVy_mz, Fz0, R0) - Mz_meas;

opts_mz = optimoptions('lsqnonlin', 'Algorithm','trust-region-reflective', ...
    'MaxIterations',50000, 'MaxFunctionEvaluations',500000, ...
    'FunctionTolerance',1e-14, 'StepTolerance',1e-14, 'Display','iter');

fprintf('  Mz マルチスタート最適化 (N=%d, 10回)...\n', length(Mz_meas));
tic;
best_mz_rn = inf; best_mz_p = mz_x0_base;
for trial = 1:10
    if trial == 1
        x0_mz = mz_x0_base;
    elseif trial <= 4
        % 物理的に合理的な範囲でバリエーション
        x0_mz = mz_x0_base;
        x0_mz(1) = 5 + 25*rand;              % qBz1
        x0_mz(4) = 0.05 + 0.4*rand;          % qDz1 (trail/R0)
        x0_mz(6) = -8 + 9*rand;              % qEz1
        x0_mz(9) = 2 + 30*rand;              % qBz9
        x0_mz(10) = -0.2 + 0.4*rand;         % qDz6
    else
        % 広範囲ランダム
        x0_mz = mz_x0_base;
        x0_mz(1) = 1 + 35*rand;
        x0_mz(3) = 0.8 + 1.4*rand;           % qCz1
        x0_mz(4) = 0.02 + 0.8*rand;
        x0_mz(6) = -9 + 10*rand;
        x0_mz(9) = 1 + 40*rand;
        x0_mz(10) = -0.3 + 0.6*rand;
        x0_mz(11) = -0.3 + 0.6*rand;
    end
    x0_mz = max(min(x0_mz, mz_ub), mz_lb);
    try
        [mz_p, mz_rn] = lsqnonlin(obj_mz, x0_mz, mz_lb, mz_ub, opts_mz);
        fprintf('    Trial %d/%d: resnorm=%.4e\n', trial, 10, mz_rn);
        if mz_rn < best_mz_rn, best_mz_rn = mz_rn; best_mz_p = mz_p; end
    catch
        fprintf('    Trial %d/%d: FAILED\n', trial, 10);
    end
end
mz_params = best_mz_p;
mz_time = toc;

% 全データ評価
Fy0_mz_all = calc_Fy0(fy_params, deg2rad(cr_pure.SA), cr_pure.Fz, deg2rad(cr_pure.IA), Fz0);
Kya_mz_all = calcKya(fy_params, cr_pure.Fz, deg2rad(cr_pure.IA), Fz0);
dfz_mz_all = (cr_pure.Fz - Fz0) ./ Fz0;
SHy_mz_all = fy_params(12) + fy_params(13) * dfz_mz_all;
SVy_mz_all = cr_pure.Fz .* (fy_params(14) + fy_params(15) * dfz_mz_all);
Mz_pred_all = calc_Mz(mz_params, deg2rad(cr_pure.SA), cr_pure.Fz, deg2rad(cr_pure.IA), ...
    Fy0_mz_all, Kya_mz_all, SHy_mz_all, SVy_mz_all, Fz0, R0);

mz_rmse = sqrt(mean((Mz_pred_all - cr_pure.Mz).^2));
mz_r2 = 1 - sum((cr_pure.Mz - Mz_pred_all).^2) / sum((cr_pure.Mz - mean(cr_pure.Mz)).^2);

fprintf('\n  ✓ Mz 完了 (%.1f sec)\n', mz_time);
fprintf('    RMSE = %.2f Nm, R² = %.4f\n', mz_rmse, mz_r2);
printParams('Mz', mz_names, mz_params);

%% ============================================================
%% Step 3: 検証
%% ============================================================
fprintf('\n=====================================================================\n');
fprintf('  検証\n');
fprintf('=====================================================================\n\n');

% --- ゼロスリップ検証 ---
fprintf('  ゼロスリップ検証 (SR=0, SA=0):\n');
fprintf('  Fz [N] | Fx [N]   | Fy [N]   | Mz [Nm]\n');
fprintf('  -------|----------|----------|--------\n');
Fz_test = [200, 500, 800, 1100, 1400];
for fz = Fz_test
    fx_z = calc_Fx0(fx_params, 0, fz, 0, Fz0);
    fy_z = calc_Fy0(fy_params, 0, fz, 0, Fz0);
    % Mz at zero-slip
    Fy0_z = fy_z;
    Kya_z = calcKya(fy_params, fz, 0, Fz0);
    dfz_z = (fz - Fz0) / Fz0;
    SHy_z = fy_params(12) + fy_params(13) * dfz_z;
    SVy_z = fz * (fy_params(14) + fy_params(15) * dfz_z);
    mz_z = calc_Mz(mz_params, 0, fz, 0, Fy0_z, Kya_z, SHy_z, SVy_z, Fz0, R0);
    fprintf('  %5d  | %8.2f | %8.2f | %7.2f\n', fz, fx_z, fy_z, mz_z);
end
fprintf('\n');

% --- SA sweep 検証 ---
fprintf('  SA sweep (SR=0, Fz=800N):\n');
fprintf('  SA [deg] | Fy [N]   | Mz [Nm]\n');
fprintf('  ---------|----------|--------\n');
sa_test = -12:2:12;
fy_sweep = zeros(size(sa_test));
mz_sweep = zeros(size(sa_test));
Kya_800 = calcKya(fy_params, 800, 0, Fz0);
dfz_800 = (800 - Fz0) / Fz0;
SHy_800 = fy_params(12) + fy_params(13) * dfz_800;
SVy_800 = 800 * (fy_params(14) + fy_params(15) * dfz_800);
for i = 1:length(sa_test)
    fy_sweep(i) = calc_Fy0(fy_params, deg2rad(sa_test(i)), 800, 0, Fz0);
    mz_sweep(i) = calc_Mz(mz_params, deg2rad(sa_test(i)), 800, 0, ...
        fy_sweep(i), Kya_800, SHy_800, SVy_800, Fz0, R0);
    fprintf('  %7.1f  | %8.1f | %7.2f\n', sa_test(i), fy_sweep(i), mz_sweep(i));
end
fprintf('  Fy range: %.1f N, Mz range: %.2f Nm\n\n', ...
    max(fy_sweep) - min(fy_sweep), max(mz_sweep) - min(mz_sweep));

% --- SR sweep 検証 ---
fprintf('  SR sweep (SA=0, Fz=800N):\n');
fprintf('  SR      | Fx [N]\n');
fprintf('  --------|--------\n');
sr_test = -0.2:0.04:0.2;
fx_sweep = zeros(size(sr_test));
for i = 1:length(sr_test)
    fx_sweep(i) = calc_Fx0(fx_params, sr_test(i), 800, 0, Fz0);
    fprintf('  %7.3f | %8.1f\n', sr_test(i), fx_sweep(i));
end
fprintf('  Fx range: %.1f N\n\n', max(fx_sweep) - min(fx_sweep));

%% ============================================================
%% Step 4: 可視化
%% ============================================================
fprintf('\nStep 4: 可視化\n\n');

% 可視化用サブサンプリング
plot_max = 5000;
cr_plot = subsample(cr_pure, plot_max);
db_plot = subsample(db_pure, plot_max);
db_raw_plot = subsample(db_raw, plot_max);

% 可視化用の予測値を再計算 (サブサンプル分のみ)
Fy_pred_plot = calc_Fy0(fy_params, deg2rad(cr_plot.SA), cr_plot.Fz, deg2rad(cr_plot.IA), Fz0);
Fx_pred_plot = calc_Fx0(fx_params, db_plot.SR, db_plot.Fz, deg2rad(db_plot.IA), Fz0);

Fx0_raw_plot = calc_Fx0(fx_params, db_raw_plot.SR, db_raw_plot.Fz, deg2rad(db_raw_plot.IA), Fz0);
Fy0_raw_plot = calc_Fy0(fy_params, deg2rad(db_raw_plot.SA), db_raw_plot.Fz, deg2rad(db_raw_plot.IA), Fz0);
Gxa_raw_plot = calc_Gxa(gxa_params, deg2rad(db_raw_plot.SA), db_raw_plot.SR, db_raw_plot.Fz, Fz0);
Gyk_raw_plot = calc_Gyk(gyk_params, deg2rad(db_raw_plot.SA), db_raw_plot.SR, db_raw_plot.Fz, Fz0);
Fx_comb_pred_plot = Gxa_raw_plot .* Fx0_raw_plot;
Fy_comb_pred_plot = Gyk_raw_plot .* Fy0_raw_plot;

Fy0_mz_plot = calc_Fy0(fy_params, deg2rad(cr_plot.SA), cr_plot.Fz, deg2rad(cr_plot.IA), Fz0);
Kya_mz_plot = calcKya(fy_params, cr_plot.Fz, deg2rad(cr_plot.IA), Fz0);
dfz_mz_plot = (cr_plot.Fz - Fz0) ./ Fz0;
SHy_mz_plot = fy_params(12) + fy_params(13) * dfz_mz_plot;
SVy_mz_plot = cr_plot.Fz .* (fy_params(14) + fy_params(15) * dfz_mz_plot);
Mz_pred_plot = calc_Mz(mz_params, deg2rad(cr_plot.SA), cr_plot.Fz, deg2rad(cr_plot.IA), ...
    Fy0_mz_plot, Kya_mz_plot, SHy_mz_plot, SVy_mz_plot, Fz0, R0);

fprintf('  可視化用サブサンプル: Cr=%d, Db純=%d, Db全=%d\n', ...
    height(cr_plot), height(db_plot), height(db_raw_plot));
fprintf('  可視化...\n');

figure('Position', [50, 50, 1800, 1200], 'Name', 'Magic Formula Fitting Results');

% (1) Fy vs SA - データ vs フィット
subplot(3,4,1);
scatter(cr_pure.SA, cr_pure.Fy, 1, cr_pure.Fz, 'filled', 'MarkerFaceAlpha', 0.1);
hold on;
sa_plot = linspace(-15, 15, 200);
for fz = [200, 500, 800, 1100, 1400]
    fy_plot = arrayfun(@(a) calc_Fy0(fy_params, deg2rad(a), fz, 0, Fz0), sa_plot);
    plot(sa_plot, fy_plot, 'k-', 'LineWidth', 1.5);
end
xlabel('SA [deg]'); ylabel('Fy [N]');
title(sprintf('Fy0: R²=%.4f', fy_r2));
colorbar; grid on;

% (2) Fx vs SR - データ vs フィット
subplot(3,4,2);
scatter(db_pure.SR, db_pure.Fx, 1, db_pure.Fz, 'filled', 'MarkerFaceAlpha', 0.1);
hold on;
sr_plot = linspace(-0.3, 0.2, 200);
for fz = [200, 500, 800, 1100, 1400]
    fx_plot = arrayfun(@(k) calc_Fx0(fx_params, k, fz, 0, Fz0), sr_plot);
    plot(sr_plot, fx_plot, 'k-', 'LineWidth', 1.5);
end
xlabel('SR'); ylabel('Fx [N]');
title(sprintf('Fx0: R²=%.4f', fx_r2));
colorbar; grid on;

% (3) Fy residual
subplot(3,4,3);
histogram(Fy_pred_all - cr_pure.Fy, 100);
xlabel('Residual [N]'); ylabel('Count');
title(sprintf('Fy0 残差: RMSE=%.1fN', fy_rmse));

% (4) Fx residual
subplot(3,4,4);
histogram(Fx_pred_all - db_pure.Fx, 100);
xlabel('Residual [N]'); ylabel('Count');
title(sprintf('Fx0 残差: RMSE=%.1fN', fx_rmse));

% (5) Fy pred vs meas
subplot(3,4,5);
scatter(cr_pure.Fy, Fy_pred_all, 1, 'filled', 'MarkerFaceAlpha', 0.1);
hold on; plot([-3000 3000], [-3000 3000], 'r--', 'LineWidth', 1);
xlabel('Fy 実測 [N]'); ylabel('Fy 予測 [N]');
title('Fy0: 予測 vs 実測'); axis equal; grid on;

% (6) Fx pred vs meas
subplot(3,4,6);
scatter(db_pure.Fx, Fx_pred_all, 1, 'filled', 'MarkerFaceAlpha', 0.1);
hold on; plot([-4000 4000], [-4000 4000], 'r--', 'LineWidth', 1);
xlabel('Fx 実測 [N]'); ylabel('Fx 予測 [N]');
title('Fx0: 予測 vs 実測'); axis equal; grid on;

% (7) 複合Fx pred vs meas
subplot(3,4,7);
scatter(db_raw.Fx, Fx_comb_pred, 1, abs(db_raw.SA), 'filled', 'MarkerFaceAlpha', 0.2);
hold on; plot([-4000 4000], [-4000 4000], 'r--', 'LineWidth', 1);
xlabel('Fx 実測 [N]'); ylabel('Fx 予測 [N]');
title(sprintf('Fx(複合): R²=%.4f', fx_comb_r2));
colorbar; axis equal; grid on;

% (8) 複合Fy pred vs meas
subplot(3,4,8);
scatter(db_raw.Fy, Fy_comb_pred, 1, abs(db_raw.SA), 'filled', 'MarkerFaceAlpha', 0.2);
hold on; plot([-3000 3000], [-3000 3000], 'r--', 'LineWidth', 1);
xlabel('Fy 実測 [N]'); ylabel('Fy 予測 [N]');
title(sprintf('Fy(複合): R²=%.4f', fy_comb_r2));
colorbar; axis equal; grid on;

% (9) Mz vs SA
subplot(3,4,9);
scatter(cr_pure.SA, cr_pure.Mz, 1, cr_pure.Fz, 'filled', 'MarkerFaceAlpha', 0.1);
hold on;
for fz = [200, 500, 800, 1100, 1400]
    Fy0_p = arrayfun(@(a) calc_Fy0(fy_params, deg2rad(a), fz, 0, Fz0), sa_plot);
    Kya_p = calcKya(fy_params, fz, 0, Fz0);
    dfz_p = (fz - Fz0) / Fz0;
    SHy_p = fy_params(12) + fy_params(13) * dfz_p;
    SVy_p = fz * (fy_params(14) + fy_params(15) * dfz_p);
    Mz_p = arrayfun(@(a, fy_val) calc_Mz(mz_params, deg2rad(a), fz, 0, ...
        fy_val, Kya_p, SHy_p, SVy_p, Fz0, R0), sa_plot, Fy0_p);
    plot(sa_plot, Mz_p, 'k-', 'LineWidth', 1.5);
end
xlabel('SA [deg]'); ylabel('Mz [Nm]');
title(sprintf('Mz: R²=%.4f', mz_r2));
colorbar; grid on;

% (10) Mz pred vs meas
subplot(3,4,10);
scatter(cr_pure.Mz, Mz_pred_all, 1, 'filled', 'MarkerFaceAlpha', 0.1);
hold on; plot([-80 80], [-80 80], 'r--', 'LineWidth', 1);
xlabel('Mz 実測 [Nm]'); ylabel('Mz 予測 [Nm]');
title(sprintf('Mz: RMSE=%.2fNm', mz_rmse));
axis equal; grid on;

% (11) Gxα - SA vs Fx減少率
subplot(3,4,11);
sa_g = linspace(-12, 12, 100);
for sr = [-0.2, -0.1, 0.1]
    gxa_plot = arrayfun(@(a) calc_Gxa(gxa_params, deg2rad(a), sr, 800, Fz0), sa_g);
    plot(sa_g, gxa_plot, 'LineWidth', 1.5, 'DisplayName', sprintf('SR=%.1f', sr));
    hold on;
end
xlabel('SA [deg]'); ylabel('Gxα');
title('Gxα: Fx低減係数'); legend; grid on;
ylim([0 1.2]);

% (12) Gyκ - SR vs Fy減少率
subplot(3,4,12);
sr_g = linspace(-0.3, 0.2, 100);
for sa = [-6, -3, 3, 6]
    gyk_plot = arrayfun(@(k) calc_Gyk(gyk_params, deg2rad(sa), k, 800, Fz0), sr_g);
    plot(sr_g, gyk_plot, 'LineWidth', 1.5, 'DisplayName', sprintf('SA=%d°', sa));
    hold on;
end
xlabel('SR'); ylabel('Gyκ');
title('Gyκ: Fy低減係数'); legend; grid on;
ylim([0 1.2]);

sgtitle(sprintf('Magic Formula Fitting: %s (Fz0=%.0fN)', ...
    strrep(tire_model, '_', '\_'), Fz0), 'FontSize', 14, 'FontWeight', 'bold');

if ~exist('output', 'dir'), mkdir('output'); end
saveas(gcf, sprintf('output/magic_formula_%s.png', tire_model));
fprintf('  ✓ 図保存\n');

%% ============================================================
%% Step 5: 保存
%% ============================================================
fprintf('\nStep 5: 保存\n\n');

mf_params = struct();
mf_params.tire_model = tire_model;
mf_params.Fz0 = Fz0;
mf_params.R0 = R0;
mf_params.Fy0 = cell2struct(num2cell(fy_params), fy_names, 2);
mf_params.Fx0 = cell2struct(num2cell(fx_params), fx_names, 2);
mf_params.Gxa = cell2struct(num2cell(gxa_params), gxa_names, 2);
mf_params.Gyk = cell2struct(num2cell(gyk_params), gyk_names, 2);
mf_params.Mz  = cell2struct(num2cell(mz_params), mz_names, 2);

mf_metrics = struct();
mf_metrics.Fy0 = struct('RMSE', fy_rmse, 'R2', fy_r2);
mf_metrics.Fx0 = struct('RMSE', fx_rmse, 'R2', fx_r2);
mf_metrics.Fx_comb = struct('RMSE', fx_comb_rmse, 'R2', fx_comb_r2);
mf_metrics.Fy_comb = struct('RMSE', fy_comb_rmse, 'R2', fy_comb_r2);
mf_metrics.Mz = struct('RMSE', mz_rmse, 'R2', mz_r2);

save_path = sprintf('output/magic_formula_%s.mat', tire_model);
save(save_path, 'mf_params', 'mf_metrics', ...
    'fy_params', 'fx_params', 'gxa_params', 'gyk_params', 'mz_params', ...
    'fy_names', 'fx_names', 'gxa_names', 'gyk_names', 'mz_names');
fprintf('  ✓ %s\n', save_path);

%% ============================================================
%% 最終サマリー
%% ============================================================
fprintf('\n=====================================================================\n');
fprintf('  フィッティング結果サマリー\n');
fprintf('=====================================================================\n\n');
fprintf('  %-20s  RMSE         R²\n', 'モデル');
fprintf('  %-20s  -----------  ------\n', '--------------------');
fprintf('  %-20s  %7.1f N    %.4f\n', '純横力 Fy0', fy_rmse, fy_r2);
fprintf('  %-20s  %7.1f N    %.4f\n', '純縦力 Fx0', fx_rmse, fx_r2);
fprintf('  %-20s  %7.1f N    %.4f\n', '複合 Fx (Gxα)', fx_comb_rmse, fx_comb_r2);
fprintf('  %-20s  %7.1f N    %.4f\n', '複合 Fy (Gyκ)', fy_comb_rmse, fy_comb_r2);
fprintf('  %-20s  %7.2f Nm   %.4f\n', 'Mz', mz_rmse, mz_r2);
fprintf('\n  パラメータ総数: %d (Fy0:%d, Fx0:%d, Gxα:%d, Gyκ:%d, Mz:%d)\n', ...
    length(fy_params)+length(fx_params)+length(gxa_params)+length(gyk_params)+length(mz_params), ...
    length(fy_params), length(fx_params), length(gxa_params), length(gyk_params), length(mz_params));
fprintf('=====================================================================\n');


%% ============================================================
%% ローカル関数
%% ============================================================

function Fy0 = calc_Fy0(p, alpha, Fz, gamma, Fz0)
    % Pacejka MF 5.2 - 純横力
    % p = [pCy1, pDy1, pDy2, pDy3, pKy1, pKy2, pKy4,
    %      pEy1, pEy2, pEy3, pEy4, pHy1, pHy2, pVy1, pVy2]
    % alpha: スリップ角 [rad], Fz: 垂直荷重 [N], gamma: キャンバ [rad]

    dfz = (Fz - Fz0) ./ Fz0;

    % 水平・垂直シフト
    SHy = p(12) + p(13) * dfz;
    SVy = Fz .* (p(14) + p(15) * dfz);

    % シフト入力
    ay = alpha + SHy;

    % 形状係数
    Cy = p(1);

    % ピーク係数 (摩擦 × 荷重)
    muy = (p(2) + p(3) * dfz) .* (1 - p(4) * gamma.^2);
    Dy = muy .* Fz;

    % コーナリング剛性
    % pKy1 の符号で Fy の方向が決まる (SAE: pKy1 < 0 → Fy < 0 for α > 0)
    Kya = p(5) * Fz0 .* sin(p(7) * atan(Fz ./ (p(6) * Fz0 + 1e-6)));

    % 剛性係数
    By = Kya ./ (Cy * Dy + sign(Dy) * 1e-6);

    % 曲率係数
    Ey = (p(8) + p(9) * dfz) .* (1 - (p(10) + p(11) * gamma) .* sign(ay));
    Ey = min(Ey, 1);  % 安定性のため E <= 1

    % Magic Formula
    By_ay = By .* ay;
    Fy0 = Dy .* sin(Cy * atan(By_ay - Ey .* (By_ay - atan(By_ay)))) + SVy;
end

function Fx0 = calc_Fx0(p, kappa, Fz, gamma, Fz0)
    % Pacejka MF 5.2 - 純縦力
    % p = [pCx1, pDx1, pDx2, pDx3, pKx1, pKx2, pKx3,
    %      pEx1, pEx2, pEx3, pEx4, pHx1, pHx2, pVx1, pVx2]
    % kappa: スリップ比 [-], Fz: 垂直荷重 [N], gamma: キャンバ [rad]

    dfz = (Fz - Fz0) ./ Fz0;

    % 水平・垂直シフト
    SHx = p(12) + p(13) * dfz;
    SVx = Fz .* (p(14) + p(15) * dfz);

    % シフト入力
    kx = kappa + SHx;

    % 形状係数
    Cx = p(1);

    % ピーク係数
    mux = (p(2) + p(3) * dfz) .* (1 - p(4) * gamma.^2);
    Dx = mux .* Fz;

    % 縦剛性
    Kxk = Fz .* (p(5) + p(6) * dfz) .* exp(p(7) * dfz);

    % 剛性係数
    Bx = Kxk ./ (Cx * Dx + sign(Dx) * 1e-6);

    % 曲率係数
    Ex = (p(8) + p(9) * dfz + p(10) * dfz.^2) .* (1 - p(11) * sign(kx));
    Ex = min(Ex, 1);

    % Magic Formula
    Bx_kx = Bx .* kx;
    Fx0 = Dx .* sin(Cx * atan(Bx_kx - Ex .* (Bx_kx - atan(Bx_kx)))) + SVx;
end

function Gxa = calc_Gxa(p, alpha, kappa, Fz, Fz0)
    % 複合スリップ: Fx 低減係数
    % p = [rBx1, rBx2, rCx1, rHx1, rEx1]
    % Fx_combined = Gxa * Fx0

    Bxa = p(1) .* cos(atan(p(2) * kappa));
    Cxa = p(3);
    as = alpha + p(4);  % shifted SA
    Exa = p(5);

    % Gxa (正規化: α=0 で Gxa=1)
    Gxa_num = cos(Cxa * atan(Bxa .* as - Exa .* (Bxa .* as - atan(Bxa .* as))));
    Gxa_den = cos(Cxa * atan(Bxa .* p(4) - Exa .* (Bxa .* p(4) - atan(Bxa .* p(4)))));

    Gxa = Gxa_num ./ (Gxa_den + 1e-10);
    Gxa = max(Gxa, 0);  % 物理的制約: Gxa >= 0
end

function Gyk = calc_Gyk(p, alpha, kappa, Fz, Fz0)
    % 複合スリップ: Fy 低減係数
    % p = [rBy1, rBy2, rBy3, rCy1, rHy1]
    % Fy_combined = Gyk * Fy0

    Byk = p(1) .* cos(atan(p(2) * (alpha - p(3))));
    Cyk = p(4);
    ks = kappa + p(5);  % shifted SR
    Eyk = 0;  % 簡易化

    % Gyk (正規化: κ=0 で Gyk=1)
    Gyk_num = cos(Cyk * atan(Byk .* ks));
    Gyk_den = cos(Cyk * atan(Byk .* p(5)));

    Gyk = Gyk_num ./ (Gyk_den + 1e-10);
    Gyk = max(Gyk, 0);
end

function Mz = calc_Mz(p, alpha, Fz, gamma, Fy0, Kya, SHy, SVy, Fz0, R0)
    % セルフアライニングトルク Mz = -trail * Fy + Mzr
    % p = [qBz1, qBz2, qCz1, qDz1, qDz2, qEz1,
    %      qHz1, qHz2, qBz9, qDz6, qDz8, qDz9]
    % R0: タイヤ無負荷半径 [m] (Dtの正規化に使用)

    dfz = (Fz - Fz0) ./ Fz0;

    % --- ニューマチックトレイル t ---
    SHt = p(7) + p(8) * dfz;
    at = alpha + SHt;

    Bt = p(1) + p(2) * dfz;
    Ct = p(3);
    % Dt = (Fz/Fz0) * R0 * (qDz1 + qDz2*dfz)  [m]
    % R0 正規化: qDz1 は無次元 (trail/R0 ≈ 0.1-0.2)
    Dt = (Fz / Fz0) .* R0 .* (p(4) + p(5) * dfz);
    Et = p(6) .* (1 - (2/pi) * sign(at) .* atan(Bt .* Ct .* at));
    Et = min(Et, 1);

    Bt_at = Bt .* at;
    trail = Dt .* cos(Ct * atan(Bt_at - Et .* (Bt_at - atan(Bt_at)))) .* cos(alpha);

    % --- 残留トルク Mzr ---
    % Dr = Fz * R0 * (qDz6 + qDz8 + qDz9*dfz)  [Nm]
    Dr = Fz .* R0 .* (p(10) + p(11) + p(12) * dfz);
    Br = p(9);
    ar = alpha + SHy + SVy ./ (Kya + sign(Kya) * 1e-6);

    Mzr = Dr .* cos(atan(Br .* ar));

    % --- 合計 ---
    Mz = -trail .* Fy0 + Mzr;
end

function Kya = calcKya(fy_params, Fz, gamma, Fz0)
    % Fy0 パラメータからコーナリング剛性を計算
    Kya = fy_params(5) * Fz0 .* sin(fy_params(7) * atan(Fz ./ (fy_params(6) * Fz0 + 1e-6)));
end

function K = estimateStiffness(slip, force, Fz, Fz0, type)
    % 初期剛性をデータから推定
    % slip: SR or SA [deg], force: Fx or Fy [N]
    
    % Fz ≈ Fz0 のデータを使用
    mask = abs(Fz - Fz0) < 0.3 * Fz0;
    
    if sum(mask) < 50
        mask = true(size(Fz));
    end
    
    s = slip(mask);
    f = force(mask);
    
    % 小スリップ領域の線形回帰
    if strcmp(type, 'lateral')
        % SA in deg → 小さいSA領域
        small = abs(s) < 3;
        if sum(small) > 20
            s_rad = deg2rad(s(small));
            f_small = f(small);
            pf = polyfit(s_rad, f_small, 1);  % 切片付き線形回帰
            K = pf(1) / Fz0;  % N/rad → 無次元化
        else
            K = -15;  % デフォルト (SAE: 負)
        end
    else
        % SR 小領域
        small = abs(s) < 0.05;
        if sum(small) > 20
            pf = polyfit(s(small), f(small), 1);  % 切片付き線形回帰
            K = pf(1) / Fz0;  % N/SR → 無次元化
        else
            K = 20;  % デフォルト
        end
    end
end

function data_sub = subsample(data, max_n)
    % ランダムサブサンプリング
    n = height(data);
    if n <= max_n
        data_sub = data;
    else
        idx = randperm(n, max_n);
        data_sub = data(idx, :);
    end
end

function printParams(name, names, vals)
    % パラメータ一覧出力
    fprintf('    %s パラメータ:\n', name);
    for i = 1:length(names)
        fprintf('      %-8s = %10.4f\n', names{i}, vals(i));
    end
end
