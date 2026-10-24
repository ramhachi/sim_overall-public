function test_rear_a_arm_optimizer
% TEST_REAR_A_ARM_OPTIMIZER - Integration test for rear A-Arm geometry optimizer
%
% 実行コマンド (MATLABで):
%   cd <project_root>/suspension_kinematics_design
%   addpath('.'); addpath('./tests');
%   test_rear_a_arm_optimizer

fprintf('=======================================================\n');
fprintf(' Test: main_rear_a_arm_optimizer\n');
fprintf('=======================================================\n');

% --- パス設定 ---
this_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(this_dir); % suspension_kinematics_design/
addpath(root_dir); % main_rear_a_arm_optimizer, a_arm_kinematics_solver
addpath(fullfile(root_dir, '..', 'matlab'));

json_file = resolve_default_vehicle_json(fullfile(root_dir, '..'), ...
    'RequireExportReady', true, ...
    'RequireSuspensionHardpoints', true, ...
    'RequiredHardpointCorners', {'RL'});

% --- 前提ファイル確認 ---
if isempty(json_file) || ~isfile(json_file)
    warning('車両 artifact が見つかりません: %s\nテストをスキップします。', string(json_file));
    return;
end

raw_before = fileread(json_file);

try
    %% Test 1: 最適化が正常完了するか
    fprintf('\n[Test 1] main_rear_a_arm_optimizer() が正常に完了するか...\n');
    orig_dir = pwd;
    cd(root_dir);
    main_rear_a_arm_optimizer(json_file);
    cd(orig_dir);
    fprintf('  -> PASS (エラーなし)\n');

    %% Test 2: Artifact に RL hardpoints が追記されているか
    fprintf('\n[Test 2] artifact params.suspension.kc.corners.RL が存在するか...\n');
    data = load_vehicle_artifact(json_file);
    assert(isfield(data.params.suspension.kc.corners, 'RL'), 'RL フィールドが見つからない');
    fprintf('  -> PASS (RL フィールド確認済み)\n');

    %% Test 3: Artifact に RR hardpoints が追記されているか
    fprintf('\n[Test 3] artifact params.suspension.kc.corners.RR が存在するか...\n');
    assert(isfield(data.params.suspension.kc.corners, 'RR'), 'RR フィールドが見つからない');
    fprintf('  -> PASS (RR フィールド確認済み)\n');

    %% Test 4: RL ロールセンターが記録された目標値の ±15mm 以内か
    fprintf('\n[Test 4] RL ロールセンター高さが記録された目標値の ±15mm 以内か...\n');
    target_RC = data.params.suspension_geometry.rear.targets.roll_center_z_m;
    tolerance = 0.015;
    rc_val = data.params.suspension.kc.rear.roll_center_height;
    fprintf('  取得値: %.4f m, 目標: %.4f m\n', rc_val, target_RC);
    assert(abs(rc_val - target_RC) <= tolerance, ...
        sprintf('ロールセンター誤差が大きすぎます: %.4f m', abs(rc_val - target_RC)));
    fprintf('  -> PASS\n');

    %% Test 5: RL KPI が記録された目標値の ±2.0 deg 以内か
    fprintf('\n[Test 5] RL KPI が記録された目標値の ±2.0 deg 以内か...\n');
    target_KPI = data.params.suspension_geometry.rear.targets.kpi_deg;
    kpi_tol = 2.0;
    kpi_val = rad2deg(data.params.suspension.kc.corners.RL.kpi);
    fprintf('  取得値: %.2f deg, 目標: %.2f deg\n', kpi_val, target_KPI);
    assert(abs(kpi_val - target_KPI) <= kpi_tol, ...
        sprintf('KPI 誤差が大きすぎます: %.2f deg', abs(kpi_val - target_KPI)));
    fprintf('  -> PASS\n');

    %% Test 6: 既存の FL フィールドが保持されているか
    fprintf('\n[Test 6] 既存の FL フィールドが保持されているか...\n');
    if isfield(data.params.suspension.kc.corners, 'FL')
        fprintf('  -> PASS (FL フィールド保持確認)\n');
    else
        fprintf('  -> INFO: FL フィールドなし (フロント最適化未実行)\n');
    end

    fprintf('\n=======================================================\n');
    fprintf(' All Tests PASSED!\n');
    fprintf('=======================================================\n');

catch ME
    cd(orig_dir);
    fid = fopen(json_file, 'w');
    fwrite(fid, raw_before, 'char');
    fclose(fid);
    fprintf('\n!!! TEST FAILED !!!\n');
    fprintf('エラー: %s\n', ME.message);
    rethrow(ME);
end

fid = fopen(json_file, 'w');
fwrite(fid, raw_before, 'char');
fclose(fid);

end
