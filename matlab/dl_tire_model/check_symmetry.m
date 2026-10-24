%% check_symmetry.m
% タイヤデータの対称性を検証:
%   横力: Fy(SA=c) ≈ -Fy(SA=-c)
%   縦力: Fx(SR=k) ≈ -Fx(SR=-k)
%   Mz:   Mz(SA=c) ≈ -Mz(SA=-c)

clear; clc;

data_dir = '../../data/tiredata/csv';
tire_model = 'Hoosier_18p0X6p0-10_R20';
Fz0 = -890;

%% Cornering データ読み込み
cr_pattern = fullfile(data_dir, ['Cr_' tire_model '*.csv']);
cr_found = dir(cr_pattern);
cr_paths = cellfun(@(f) fullfile(data_dir, f), sort({cr_found.name}'), 'UniformOutput', false);
cr_raw = loadTireData(cr_paths);
cr_pure = cr_raw(abs(cr_raw.SR) < 0.05, :);

fprintf('=== 対称性検証 ===\n\n');

%% --- 1. Cornering Fy: SA分布 ---
fprintf('--- Cornering SA分布 ---\n');
fprintf('SA > 0.5°  : %d 点\n', sum(cr_pure.SA > 0.5));
fprintf('SA < -0.5° : %d 点\n', sum(cr_pure.SA < -0.5));
fprintf('|SA| < 0.5°: %d 点\n\n', sum(abs(cr_pure.SA) < 0.5));

%% --- 2. Fy(SA) vs -Fy(-SA) at Fz≈Fz0 ---
fprintf('--- Fy 対称性 (Fz≈%.0f±200N) ---\n', Fz0);
fz_mask = abs(cr_pure.Fz - Fz0) < 200;
cr_ref = cr_pure(fz_mask, :);
fprintf('データ数: %d 点\n\n', height(cr_ref));

sa_bins = [-12 -10 -8 -6 -4 -2 2 4 6 8 10 12];
fprintf('%6s | %8s %5s | %9s %5s | %8s\n', 'SA', 'Fy', 'N', '-Fy(-SA)', 'N', '差');
fprintf('-------|----------------|-----------------|--------\n');
for j = 1:length(sa_bins)
    sa = sa_bins(j);
    m1 = abs(cr_ref.SA - sa) < 0.5;
    m2 = abs(cr_ref.SA + sa) < 0.5;
    if sum(m1) > 0 && sum(m2) > 0
        fy1 = mean(cr_ref.Fy(m1));
        fy2 = mean(cr_ref.Fy(m2));
        % 対称なら Fy(SA) ≈ -Fy(-SA)
        fprintf('%6.1f | %8.1f %5d | %9.1f %5d | %8.1f\n', ...
            sa, fy1, sum(m1), -fy2, sum(m2), fy1 - (-fy2));
    end
end

%% --- 3. Mz対称性 ---
fprintf('\n--- Mz 対称性 (Fz≈%.0f±200N) ---\n', Fz0);
fprintf('%6s | %8s %5s | %9s %5s | %8s\n', 'SA', 'Mz', 'N', '-Mz(-SA)', 'N', '差');
fprintf('-------|----------------|-----------------|--------\n');
for j = 1:length(sa_bins)
    sa = sa_bins(j);
    m1 = abs(cr_ref.SA - sa) < 0.5;
    m2 = abs(cr_ref.SA + sa) < 0.5;
    if sum(m1) > 0 && sum(m2) > 0
        mz1 = mean(cr_ref.Mz(m1));
        mz2 = mean(cr_ref.Mz(m2));
        fprintf('%6.1f | %8.2f %5d | %9.2f %5d | %8.2f\n', ...
            sa, mz1, sum(m1), -mz2, sum(m2), mz1 - (-mz2));
    end
end

%% --- 4. DriveBrake データ ---
db_pattern = fullfile(data_dir, ['Db_' tire_model '*.csv']);
db_found = dir(db_pattern);
db_all = sort({db_found.name}');
db_keep = db_all(~contains(db_all, 'Run068') & ~contains(db_all, 'Run071'));
db_paths = cellfun(@(f) fullfile(data_dir, f), db_keep, 'UniformOutput', false);
db_raw = loadTireData(db_paths);
db_pure = db_raw(abs(db_raw.SA) < 1, :);

fprintf('\n--- DriveBrake SR分布 (|SA|<1°) ---\n');
fprintf('SR > 0.01  : %d 点\n', sum(db_pure.SR > 0.01));
fprintf('SR < -0.01 : %d 点\n', sum(db_pure.SR < -0.01));
fprintf('|SR| < 0.01: %d 点\n\n', sum(abs(db_pure.SR) < 0.01));

fprintf('--- Fx 対称性 (Fz≈%.0f±200N, |SA|<1°) ---\n', Fz0);
fz_mask2 = abs(db_pure.Fz - Fz0) < 200;
db_ref = db_pure(fz_mask2, :);
fprintf('データ数: %d 点\n\n', height(db_ref));

sr_bins = [0.05 0.10 0.15 0.20];
fprintf('%7s | %8s %5s | %9s %5s | %8s\n', 'SR', 'Fx', 'N', '-Fx(-SR)', 'N', '差');
fprintf('--------|----------------|-----------------|--------\n');
for j = 1:length(sr_bins)
    sr = sr_bins(j);
    m1 = abs(db_ref.SR - sr) < 0.015;
    m2 = abs(db_ref.SR + sr) < 0.015;
    if sum(m1) > 5 && sum(m2) > 5
        fx1 = mean(db_ref.Fx(m1));
        fx2 = mean(db_ref.Fx(m2));
        fprintf('%7.3f | %8.1f %5d | %9.1f %5d | %8.1f\n', ...
            sr, fx1, sum(m1), -fx2, sum(m2), fx1 - (-fx2));
    else
        fprintf('%7.3f | データ不足 (SR+: %d, SR-: %d)\n', sr, sum(m1), sum(m2));
    end
end

fprintf('\n=== 完了 ===\n');
