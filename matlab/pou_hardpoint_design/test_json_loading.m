% TEST_JSON_LOADING - JSONファイルの読み込みテスト
%
% optimized_autocross.jsonが正しく読み込めるか確認

clear; clc;

this_dir = fileparts(mfilename('fullpath'));
addpath(fileparts(this_dir));

fprintf('=== JSON読み込みテスト ===\n\n');

% JSONファイルのパス
json_file = '../optimized_autocross.json';

fprintf('1. JSONファイルの存在確認...\n');
if exist(json_file, 'file')
    fprintf('   ✓ ファイル発見: %s\n\n', json_file);
else
    fprintf('   ✗ ファイルが見つかりません: %s\n', json_file);
    return;
end

% JSONファイルを読み込み
fprintf('2. JSONファイルを読み込んでいます...\n');
try
    json_data = load_vehicle_artifact(json_file);
    params = json_data.params;
    fprintf('   ✓ 読み込み成功\n\n');
catch ME
    fprintf('   ✗ 読み込み失敗: %s\n', ME.message);
    return;
end

% 構造を確認
fprintf('3. JSON構造を確認...\n');
if isfield(json_data, 'params')
    fprintf('   ✓ トップレベルに "params" フィールドあり\n');

    if isfield(params, 'suspension')
        fprintf('   ✓ params.suspension フィールドあり\n');

        if isfield(params.suspension, 'pou_jack_c1')
            fprintf('   ✓ params.suspension.pou_jack_c1 フィールドあり\n\n');
        else
            fprintf('   ✗ params.suspension.pou_jack_c1 フィールドなし\n');
            return;
        end
    else
        fprintf('   ✗ params.suspension フィールドなし\n');
        return;
    end
else
    fprintf('   ⚠ トップレベルに "params" フィールドなし\n');
end

% POUパラメータを抽出
fprintf('4. POUパラメータを抽出...\n');
try
    susp = params.suspension;

    fprintf('   pou_jack_c1:    [%.4f, %.4f, %.4f, %.4f]\n', susp.pou_jack_c1);
    fprintf('   pou_jack_c2:    [%.4f, %.4f, %.4f, %.4f]\n', susp.pou_jack_c2);
    fprintf('   pou_camber_c1:  [%.4f, %.4f, %.4f, %.4f]\n', susp.pou_camber_c1);
    fprintf('   pou_camber_c2:  [%.4f, %.4f, %.4f, %.4f]\n\n', susp.pou_camber_c2);

    fprintf('   ✓ POUパラメータ抽出成功\n\n');
catch ME
    fprintf('   ✗ 抽出失敗: %s\n', ME.message);
    return;
end

% 車両パラメータを抽出
fprintf('5. 車両パラメータを抽出...\n');
try
    mass_params = params.mass;

    fprintf('   wheelbase:  %.3f m\n', mass_params.wheelbase);
    fprintf('   track_f:    %.3f m\n', mass_params.track_f);
    fprintf('   track_r:    %.3f m\n\n', mass_params.track_r);

    fprintf('   ✓ 車両パラメータ抽出成功\n\n');
catch ME
    fprintf('   ✗ 抽出失敗: %s\n', ME.message);
    return;
end

% FLホイールの目標多項式を構築
fprintf('6. FLホイールの目標多項式を構築...\n');
idx = 1;  % FL wheel (MATLAB 1-based indexing)

target_poly.jack_c1 = susp.pou_jack_c1(idx);
target_poly.jack_c2 = susp.pou_jack_c2(idx);
target_poly.camber_c1 = susp.pou_camber_c1(idx);
target_poly.camber_c2 = susp.pou_camber_c2(idx);

fprintf('   FLホイール目標多項式:\n');
fprintf('     Jack-up:  c1 = %.4f m/rad, c2 = %.4f m/rad²\n', ...
    target_poly.jack_c1, target_poly.jack_c2);
fprintf('     Camber:   c1 = %.4f rad/rad, c2 = %.4f rad/rad²\n\n', ...
    target_poly.camber_c1, target_poly.camber_c2);

fprintf('   ✓ 目標多項式構築成功\n\n');

fprintf('=== すべてのテスト合格 ===\n');
fprintf('main_pou_hardpoint_optimizer() を実行する準備ができました。\n\n');

fprintf('次のコマンドを実行してください:\n');
fprintf('  results = main_pou_hardpoint_optimizer(''%s'', struct(''wheel_index'', 0));\n', json_file);
