%% test_aero_map_4d.m
% 4Dエアロマップテスト

clear; clc; close all;

fprintf('==========================================\n');
fprintf(' 4D Aero Map Test\n');
fprintf('==========================================\n\n');

% パスを追加
addpath('../aero');

% エアロマップ作成
aero = aero_map_4d();

% グリッド情報
fprintf('Grid dimensions:\n');
fprintf('  RH_front: %d points (%.0f-%.0f mm)\n', ...
    length(aero.grid_rh_f), aero.grid_rh_f(1), aero.grid_rh_f(end));
fprintf('  RH_rear:  %d points (%.0f-%.0f mm)\n', ...
    length(aero.grid_rh_r), aero.grid_rh_r(1), aero.grid_rh_r(end));
fprintf('  Roll:     %d points (%.1f-%.1f deg)\n', ...
    length(aero.grid_roll), rad2deg(aero.grid_roll(1)), rad2deg(aero.grid_roll(end)));
fprintf('  Yaw:      %d points (%.1f-%.1f deg)\n', ...
    length(aero.grid_yaw), rad2deg(aero.grid_yaw(1)), rad2deg(aero.grid_yaw(end)));

%% テスト1: ベースライン
fprintf('\n=== Test 1: Baseline Condition ===\n');
rh_f = 40; rh_r = 55; roll = 0; yaw = 0;
velocity = 20;

coef = aero.get_coeffs(rh_f, rh_r, roll, yaw);
forces = aero.compute_forces(velocity, rh_f, rh_r, roll, yaw);

fprintf('RH_f=%.0fmm, RH_r=%.0fmm, roll=%.1fdeg, yaw=%.1fdeg, V=%.0fm/s\n', ...
    rh_f, rh_r, rad2deg(roll), rad2deg(yaw), velocity);
fprintf('Coefficients: CL=%.3f, CD=%.3f, CY=%.3f\n', coef.CL, coef.CD, coef.CY);
fprintf('CoP_x=%.3fm, CoP_dist=%.2f\n', coef.CoP_x, coef.CoP_dist);
fprintf('Forces: Fz=%.0fN, Fx=%.0fN, Fy=%.0fN\n', forces.Fz, forces.Fx, forces.Fy);
fprintf('  Fz_front=%.0fN, Fz_rear=%.0fN\n', forces.Fz_front, forces.Fz_rear);

%% テスト2: ライドハイト効果
fprintf('\n=== Test 2: Ride Height Effect ===\n');
rh_test = [25, 35, 45, 55];
for rh = rh_test
    c = aero.get_coeffs(rh, 55, 0, 0);
    f = aero.compute_forces(velocity, rh, 55, 0, 0);
    fprintf('RH_f=%.0fmm: CL=%.3f, Fz=%.0fN\n', rh, c.CL, f.Fz);
end

%% テスト3: ロール効果
fprintf('\n=== Test 3: Roll Effect ===\n');
roll_test = [0, deg2rad(2.5), deg2rad(5)];
coef_base = aero.get_coeffs(40, 55, 0, 0);
for r = roll_test
    c = aero.get_coeffs(40, 55, r, 0);
    loss = (1 + c.CL/coef_base.CL) * 100;
    fprintf('Roll=%.1fdeg: CL=%.3f (loss=%.1f%%)\n', rad2deg(r), c.CL, loss);
end

%% テスト4: ヨー効果
fprintf('\n=== Test 4: Yaw Effect ===\n');
yaw_test = deg2rad([0, 5, 10, 15]);
for y = yaw_test
    c = aero.get_coeffs(40, 55, 0, y);
    fprintf('Yaw=%.1fdeg: CL=%.3f, CD=%.3f, CY=%.3f\n', rad2deg(y), c.CL, c.CD, c.CY);
end

%% テスト5: 速度スイープ
fprintf('\n=== Test 5: Velocity Sweep ===\n');
v_test = [10, 15, 20, 25, 30];
for v = v_test
    f = aero.compute_forces(v, 40, 55, 0, 0);
    fprintf('V=%.0fm/s: Fz=%.0fN, Fx=%.0fN\n', v, f.Fz, f.Fx);
end

%% プロット
figure('Position', [50, 50, 1400, 800]);

% ライドハイトvsダウンフォース
subplot(2,3,1);
rh_f_vec = linspace(20, 60, 20);
CL_vec = zeros(size(rh_f_vec));
for i = 1:length(rh_f_vec)
    c = aero.get_coeffs(rh_f_vec(i), 55, 0, 0);
    CL_vec(i) = c.CL;
end
plot(rh_f_vec, CL_vec, 'b-', 'LineWidth', 2);
xlabel('Front Ride Height [mm]');
ylabel('CL');
title('Ground Effect (Rear RH=55mm)');
grid on;

% ロール vs CL
subplot(2,3,2);
roll_vec = deg2rad(linspace(-5, 5, 20));
CL_roll = zeros(size(roll_vec));
for i = 1:length(roll_vec)
    c = aero.get_coeffs(40, 55, roll_vec(i), 0);
    CL_roll(i) = c.CL;
end
plot(rad2deg(roll_vec), CL_roll, 'b-', 'LineWidth', 2);
xlabel('Roll Angle [deg]');
ylabel('CL');
title('Roll Effect on Downforce');
grid on;

% ヨー vs CL/CD
subplot(2,3,3);
yaw_vec = deg2rad(linspace(-15, 15, 30));
CL_yaw = zeros(size(yaw_vec));
CD_yaw = zeros(size(yaw_vec));
CY_yaw = zeros(size(yaw_vec));
for i = 1:length(yaw_vec)
    c = aero.get_coeffs(40, 55, 0, yaw_vec(i));
    CL_yaw(i) = c.CL;
    CD_yaw(i) = c.CD;
    CY_yaw(i) = c.CY;
end
yyaxis left;
plot(rad2deg(yaw_vec), CL_yaw, 'b-', 'LineWidth', 2);
ylabel('CL');
yyaxis right;
plot(rad2deg(yaw_vec), CD_yaw, 'r-', 'LineWidth', 2);
ylabel('CD');
xlabel('Yaw Angle [deg]');
title('Yaw Effect');
legend('CL', 'CD');
grid on;

% サイドフォース
subplot(2,3,4);
plot(rad2deg(yaw_vec), CY_yaw, 'b-', 'LineWidth', 2);
xlabel('Yaw Angle [deg]');
ylabel('CY');
title('Side Force Coefficient');
grid on;

% 速度 vs Force
subplot(2,3,5);
v_vec = linspace(5, 35, 30);
Fz_vec = zeros(size(v_vec));
Fx_vec = zeros(size(v_vec));
for i = 1:length(v_vec)
    f = aero.compute_forces(v_vec(i), 40, 55, 0, 0);
    Fz_vec(i) = f.Fz;
    Fx_vec(i) = f.Fx;
end
yyaxis left;
plot(v_vec, -Fz_vec, 'b-', 'LineWidth', 2);
ylabel('Downforce [N]');
yyaxis right;
plot(v_vec, -Fx_vec, 'r-', 'LineWidth', 2);
ylabel('Drag [N]');
xlabel('Velocity [m/s]');
title('Aero Forces vs Velocity');
legend('Downforce', 'Drag');
grid on;

% 2D RH マップ
subplot(2,3,6);
[RH_F, RH_R] = meshgrid(aero.grid_rh_f, aero.grid_rh_r);
CL_map = zeros(size(RH_F));
for i = 1:size(RH_F, 1)
    for j = 1:size(RH_F, 2)
        c = aero.get_coeffs(RH_F(i,j), RH_R(i,j), 0, 0);
        CL_map(i,j) = c.CL;
    end
end
contourf(RH_F, RH_R, CL_map, 20);
colorbar;
xlabel('Front RH [mm]');
ylabel('Rear RH [mm]');
title('CL Map (roll=0, yaw=0)');

sgtitle('4D Aero Map Verification');

saveas(gcf, '../../output/aero_map_4d_test.png');
fprintf('\nSaved plot to output/aero_map_4d_test.png\n');

%% 検証
fprintf('\n=== Verification ===\n');
passed = true;

% CL範囲チェック
if coef.CL < -1 && coef.CL > -4
    fprintf('✓ CL in realistic range: %.3f\n', coef.CL);
else
    fprintf('✗ CL out of range: %.3f\n', coef.CL);
    passed = false;
end

% 地面効果チェック
c_low = aero.get_coeffs(25, 40, 0, 0);
c_high = aero.get_coeffs(55, 75, 0, 0);
if c_low.CL < c_high.CL
    fprintf('✓ Ground effect correct: low RH gives more DF\n');
else
    fprintf('✗ Ground effect incorrect\n');
    passed = false;
end

% ダウンフォースは負
if forces.Fz < 0
    fprintf('✓ Downforce is negative: %.0f N\n', forces.Fz);
else
    fprintf('✗ Fz should be negative\n');
    passed = false;
end

if passed
    fprintf('\n4D Aero Map test PASSED\n');
else
    fprintf('\n4D Aero Map test FAILED\n');
end

fprintf('\n==========================================\n');
