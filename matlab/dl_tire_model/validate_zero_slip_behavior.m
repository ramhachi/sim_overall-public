%% validate_zero_slip_behavior.m
% 学習済みモデルのゼロスリップ挙動を検証
%
% 目的:
%   - SA=0, SR=0 で Fx, Fy, Mz が正しくゼロ近傍になるか確認
%   - 分離学習により以前のバイアス問題が解決されているか検証
%
% 使用方法:
%   1. tire_model に学習済みモデル名を設定
%   2. MATLABで実行

clear; clc; close all;





fprintf('\n');
fprintf('=================================================================\n');
fprintf(' Zero-Slip Behavior Validation\n');
fprintf(' ゼロスリップ時の挙動検証\n');
fprintf('=================================================================\n\n');

%% ============================================================
%% ユーザ設定
%% ============================================================

% タイヤモデル名（学習済み.matファイルの識別子）
tire_model = 'Hoosier_18p0X6p0-10_R20';

% 検証条件
test_Fz = [200, 500, 800, 1100, 1400];  % 荷重 [N]
test_IA = [0, 2, -2];                    % キャンバー角 [deg]
test_TSTC = 30;                          % タイヤ表面温度 [℃]

%% ============================================================
%% モデル読み込み
%% ============================================================
fprintf('学習済みモデルを読み込み中...\n');

cr_file = sprintf('output/cornering_%s.mat', tire_model);
db_file = sprintf('output/drivebrake_%s.mat', tire_model);

if ~exist(cr_file, 'file')
    error('Corneringモデルが見つかりません: %s', cr_file);
end
if ~exist(db_file, 'file')
    error('DriveBrakeモデルが見つかりません: %s', db_file);
end

cr_model = load(cr_file);
db_model = load(db_file);

fprintf('  ✓ Cornering model: %s\n', cr_file);
fprintf('  ✓ DriveBrake model: %s\n', db_file);
fprintf('  タイヤ: %s\n\n', tire_model);

%% ============================================================
%% Test 1: SA=0, SR=0 でゼロ出力確認
%% ============================================================
fprintf('=================================================================\n');
fprintf(' Test 1: SA=0, SR=0 → すべての力/モーメントがゼロ近傍であるべき\n');
fprintf('=================================================================\n\n');

n_test = length(test_Fz) * length(test_IA);
test_results_zero = struct();
test_results_zero.Fz = [];
test_results_zero.IA = [];
test_results_zero.cr_Fx = [];
test_results_zero.cr_Fy = [];
test_results_zero.cr_Mz = [];
test_results_zero.db_Fx = [];
test_results_zero.db_Fy = [];
test_results_zero.db_Mz = [];

fprintf('条件: SA=0, SR=0, TSTC=%.1f℃\n', test_TSTC);
fprintf('-------------------------------------------------------------\n');
fprintf('   Fz [N] |  IA [deg] | Cr:Fx [N] | Cr:Fy [N] | Cr:Mz [Nm] | Db:Fx [N] | Db:Fy [N]\n');
fprintf('-------------------------------------------------------------\n');

tic_test1 = tic;
for i = 1:length(test_Fz)
    for j = 1:length(test_IA)
        Fz = test_Fz(i);
        IA = test_IA(j);
        
        % 入力作成 [SR, SA, Fz, IA, TSTC]
        X_zero = [0, 0, Fz, IA, test_TSTC];
        
        % Cornering model で予測
        X_cr_norm = (X_zero - cr_model.cr_norm.input_mean) ./ cr_model.cr_norm.input_std;
        Y_cr_norm = predict(cr_model.cr_trained, X_cr_norm);
        Y_cr = Y_cr_norm .* cr_model.cr_norm.output_std + cr_model.cr_norm.output_mean;
        
        % DriveBrake model で予測
        X_db_norm = (X_zero - db_model.db_norm.input_mean) ./ db_model.db_norm.input_std;
        Y_db_norm = predict(db_model.db_trained, X_db_norm);
        Y_db = Y_db_norm .* db_model.db_norm.output_std + db_model.db_norm.output_mean;
        
        % 結果保存
        test_results_zero.Fz(end+1) = Fz;
        test_results_zero.IA(end+1) = IA;
        test_results_zero.cr_Fx(end+1) = Y_cr(1);
        test_results_zero.cr_Fy(end+1) = Y_cr(2);
        test_results_zero.cr_Mz(end+1) = Y_cr(4);
        test_results_zero.db_Fx(end+1) = Y_db(1);
        test_results_zero.db_Fy(end+1) = Y_db(2);
        test_results_zero.db_Mz(end+1) = Y_db(4);
        
        % 表示
        fprintf(' %8.0f | %9.1f | %9.2f | %9.2f | %10.2f | %9.2f | %9.2f\n', ...
            Fz, IA, Y_cr(1), Y_cr(2), Y_cr(4), Y_db(1), Y_db(2));
    end
end
test1_time = toc(tic_test1);

fprintf('-------------------------------------------------------------\n');
fprintf('予測時間: %.4f 秒（%d サンプル）\n', test1_time, n_test);
fprintf('平均速度: %.2f ms/サンプル, %.0f サンプル/秒\n\n', ...
    test1_time*1000/n_test, n_test/test1_time);

% 統計情報
fprintf('統計情報（ゼロスリップ時の出力）:\n');
fprintf('  Cornering model:\n');
fprintf('    Fx: mean=%.2fN, std=%.2fN, max=%.2fN\n', ...
    mean(abs(test_results_zero.cr_Fx)), std(test_results_zero.cr_Fx), max(abs(test_results_zero.cr_Fx)));
fprintf('    Fy: mean=%.2fN, std=%.2fN, max=%.2fN\n', ...
    mean(abs(test_results_zero.cr_Fy)), std(test_results_zero.cr_Fy), max(abs(test_results_zero.cr_Fy)));
fprintf('    Mz: mean=%.2fNm, std=%.2fNm, max=%.2fNm\n', ...
    mean(abs(test_results_zero.cr_Mz)), std(test_results_zero.cr_Mz), max(abs(test_results_zero.cr_Mz)));

fprintf('\n  DriveBrake model:\n');
fprintf('    Fx: mean=%.2fN, std=%.2fN, max=%.2fN\n', ...
    mean(abs(test_results_zero.db_Fx)), std(test_results_zero.db_Fx), max(abs(test_results_zero.db_Fx)));
fprintf('    Fy: mean=%.2fN, std=%.2fN, max=%.2fN\n', ...
    mean(abs(test_results_zero.db_Fy)), std(test_results_zero.db_Fy), max(abs(test_results_zero.db_Fy)));
fprintf('    Mz: mean=%.2fNm, std=%.2fNm, max=%.2fNm\n\n', ...
    mean(abs(test_results_zero.db_Mz)), std(test_results_zero.db_Mz), max(abs(test_results_zero.db_Mz)));

% 判定
threshold_F = 50;  % 力の許容誤差 [N]
threshold_M = 5;   % モーメントの許容誤差 [Nm]

cr_fx_ok = max(abs(test_results_zero.cr_Fx)) < threshold_F;
cr_fy_ok = max(abs(test_results_zero.cr_Fy)) < threshold_F;
db_fx_ok = max(abs(test_results_zero.db_Fx)) < threshold_F;
db_fy_ok = max(abs(test_results_zero.db_Fy)) < threshold_F;

fprintf('判定（閾値: 力=%.0fN, モーメント=%.0fNm）:\n', threshold_F, threshold_M);
fprintf('  Cornering Fx: %s\n', judge_str(cr_fx_ok));
fprintf('  Cornering Fy: %s\n', judge_str(cr_fy_ok));
fprintf('  DriveBrake Fx: %s\n', judge_str(db_fx_ok));
fprintf('  DriveBrake Fy: %s\n\n', judge_str(db_fy_ok));

%% ============================================================
%% Test 2: SA=0, SR変化 → Fxのみ変化、Fyはゼロ近傍
%% ============================================================
fprintf('=================================================================\n');
fprintf(' Test 2: SA=0, SR変化 → Fxのみ変化、Fyはゼロ近傍（DriveBrake特性）\n');
fprintf('=================================================================\n\n');

SR_range = linspace(-0.2, 0.2, 41);
test_Fz_fixed = 800;  % N
test_IA_fixed = 0;    % deg

X_test2 = zeros(length(SR_range), 5);
for i = 1:length(SR_range)
    X_test2(i, :) = [SR_range(i), 0, test_Fz_fixed, test_IA_fixed, test_TSTC];
end

% DriveBrake model で予測
tic_test2 = tic;
X_db_norm2 = (X_test2 - db_model.db_norm.input_mean) ./ db_model.db_norm.input_std;
Y_db_norm2 = predict(db_model.db_trained, X_db_norm2);
Y_db2 = Y_db_norm2 .* db_model.db_norm.output_std + db_model.db_norm.output_mean;
test2_time = toc(tic_test2);

fprintf('条件: SA=0, Fz=%.0fN, IA=%.1fdeg, TSTC=%.1f℃\n', test_Fz_fixed, test_IA_fixed, test_TSTC);
fprintf('SR範囲: %.2f ～ %.2f\n\n', min(SR_range), max(SR_range));

fprintf('  Fx: min=%.1fN, max=%.1fN\n', min(Y_db2(:,1)), max(Y_db2(:,1)));
fprintf('  Fy: mean=%.2fN, std=%.2fN, max=%.2fN\n', ...
    mean(abs(Y_db2(:,2))), std(Y_db2(:,2)), max(abs(Y_db2(:,2))));

fprintf('予測時間: %.4f 秒（%d サンプル）\n', test2_time, length(SR_range));
fprintf('平均速度: %.2f ms/サンプル, %.0f サンプル/秒\n\n', ...
    test2_time*1000/length(SR_range), length(SR_range)/test2_time);

fy_ok_test2 = max(abs(Y_db2(:,2))) < threshold_F;
fprintf('\n判定（Fyがゼロ近傍か）: %s\n\n', judge_str(fy_ok_test2));

%% ============================================================
%% Test 3: SR=0, SA変化 → Fyのみ変化、Fxはゼロ近傍
%% ============================================================
fprintf('=================================================================\n');
fprintf(' Test 3: SR=0, SA変化 → Fyのみ変化、Fxはゼロ近傍（Cornering特性）\n');
fprintf('=================================================================\n\n');

SA_range = linspace(-12, 12, 49);

X_test3 = zeros(length(SA_range), 5);
for i = 1:length(SA_range)
    X_test3(i, :) = [0, SA_range(i), test_Fz_fixed, test_IA_fixed, test_TSTC];
end

% Cornering model で予測
tic_test3 = tic;
X_cr_norm3 = (X_test3 - cr_model.cr_norm.input_mean) ./ cr_model.cr_norm.input_std;
Y_cr_norm3 = predict(cr_model.cr_trained, X_cr_norm3);
Y_cr3 = Y_cr_norm3 .* cr_model.cr_norm.output_std + cr_model.cr_norm.output_mean;
test3_time = toc(tic_test3);

fprintf('条件: SR=0, Fz=%.0fN, IA=%.1fdeg, TSTC=%.1f℃\n', test_Fz_fixed, test_IA_fixed, test_TSTC);
fprintf('SA範囲: %.1f ～ %.1f deg\n\n', min(SA_range), max(SA_range));

fprintf('  Fy: min=%.1fN, max=%.1fN\n', min(Y_cr3(:,2)), max(Y_cr3(:,2)));
fprintf('  Fx: mean=%.2fN, std=%.2fN, max=%.2fN\n', ...
    mean(abs(Y_cr3(:,1))), std(Y_cr3(:,1)), max(abs(Y_cr3(:,1))));

fprintf('予測時間: %.4f 秒（%d サンプル）\n', test3_time, length(SA_range));
fprintf('平均速度: %.2f ms/サンプル, %.0f サンプル/秒\n\n', ...
    test3_time*1000/length(SA_range), length(SA_range)/test3_time);

fx_ok_test3 = max(abs(Y_cr3(:,1))) < threshold_F;
fprintf('\n判定（Fxがゼロ近傍か）: %s\n\n', judge_str(fx_ok_test3));

%% ============================================================
%% 可視化
%% ============================================================
fprintf('=================================================================\n');
fprintf(' 可視化\n');
fprintf('=================================================================\n\n');

figure('Position', [100, 100, 1400, 900]);

% Test 2: SA=0, SR変化
subplot(2, 3, 1);
plot(SR_range*100, Y_db2(:,1), 'b-', 'LineWidth', 2);
grid on;
xlabel('Slip Ratio SR [%]');
ylabel('Fx [N]');
title('Test 2: DriveBrake特性（SA=0）');
subtitle(sprintf('Fz=%.0fN, IA=%.1fdeg', test_Fz_fixed, test_IA_fixed));

subplot(2, 3, 2);
plot(SR_range*100, Y_db2(:,2), 'r-', 'LineWidth', 2);
hold on;
yline(threshold_F, 'k--', sprintf('閾値 %.0fN', threshold_F));
yline(-threshold_F, 'k--');
grid on;
xlabel('Slip Ratio SR [%]');
ylabel('Fy [N]');
title('Fy（ゼロ近傍であるべき）');
if fy_ok_test2
    subtitle('✓ PASS');
else
    subtitle('✗ FAIL');
end

subplot(2, 3, 3);
plot(SR_range*100, Y_db2(:,4), 'g-', 'LineWidth', 2);
grid on;
xlabel('Slip Ratio SR [%]');
ylabel('Mz [Nm]');
title('Mz（参考）');

% Test 3: SR=0, SA変化
subplot(2, 3, 4);
plot(SA_range, Y_cr3(:,1), 'b-', 'LineWidth', 2);
hold on;
yline(threshold_F, 'k--', sprintf('閾値 %.0fN', threshold_F));
yline(-threshold_F, 'k--');
grid on;
xlabel('Slip Angle SA [deg]');
ylabel('Fx [N]');
title('Fx（ゼロ近傍であるべき）');
if fx_ok_test3
    subtitle('✓ PASS');
else
    subtitle('✗ FAIL');
end

subplot(2, 3, 5);
plot(SA_range, Y_cr3(:,2), 'r-', 'LineWidth', 2);
grid on;
xlabel('Slip Angle SA [deg]');
ylabel('Fy [N]');
title('Test 3: Cornering特性（SR=0）');
subtitle(sprintf('Fz=%.0fN, IA=%.1fdeg', test_Fz_fixed, test_IA_fixed));

subplot(2, 3, 6);
plot(SA_range, Y_cr3(:,4), 'g-', 'LineWidth', 2);
grid on;
xlabel('Slip Angle SA [deg]');
ylabel('Mz [Nm]');
title('Mz (Self-Aligning Torque)');

sgtitle(sprintf('Zero-Slip Behavior Validation: %s', strrep(tire_model, '_', '\_')), ...
    'FontSize', 14, 'FontWeight', 'bold');

% 保存
if ~exist('output', 'dir'), mkdir('output'); end
saveas(gcf, sprintf('output/zero_slip_validation_%s.png', tire_model));
fprintf('  ✓ 図を保存: output/zero_slip_validation_%s.png\n\n', tire_model);

%% ============================================================
%% 最終判定
%% ============================================================
fprintf('=================================================================\n');
fprintf(' 最終判定\n');
fprintf('=================================================================\n\n');

all_pass = cr_fx_ok && cr_fy_ok && db_fx_ok && db_fy_ok && fy_ok_test2 && fx_ok_test3;

fprintf('Test 1 (SA=0, SR=0でゼロ出力):\n');
fprintf('  Cornering Fx: %s\n', judge_str(cr_fx_ok));
fprintf('  Cornering Fy: %s\n', judge_str(cr_fy_ok));
fprintf('  DriveBrake Fx: %s\n', judge_str(db_fx_ok));
fprintf('  DriveBrake Fy: %s\n', judge_str(db_fy_ok));
fprintf('  予測時間: %.4f 秒\n', test1_time);
fprintf('\nTest 2 (SA=0, SR変化でFyゼロ近傍): %s\n', judge_str(fy_ok_test2));
fprintf('  予測時間: %.4f 秒\n', test2_time);
fprintf('\nTest 3 (SR=0, SA変化でFxゼロ近傍): %s\n', judge_str(fx_ok_test3));
fprintf('  予測時間: %.4f 秒\n\n', test3_time);

if all_pass
    fprintf('========================================\n');
    fprintf('   ✓ 全テスト PASS\n');
    fprintf('   分離学習により正常な挙動を確認\n');
    fprintf('========================================\n\n');
else
    fprintf('========================================\n');
    fprintf('   ⚠ 一部テスト FAIL\n');
    fprintf('   詳細は上記の結果を確認\n');
    fprintf('========================================\n\n');
end

%% ============================================================
%% ローカル関数
%% ============================================================

function str = judge_str(pass)
    if pass
        str = '✓ PASS';
    else
        str = '✗ FAIL';
    end
end
