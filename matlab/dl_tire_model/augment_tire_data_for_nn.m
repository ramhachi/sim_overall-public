%% augment_tire_data_for_nn.m
% NNタイヤモデル訓練データの拡張
% 問題点の解決:
% 1. ゼロスリップ点の追加（SR=0, SA=0でFx=Fy=0）
% 2. 対称性の強制（Fy(SA) = -Fy(-SA), Fx(SR) = -Fx(-SR)）

clear; clc;

fprintf('=================================================================\n');
fprintf(' Tire Data Augmentation for NN Training\n');
fprintf('=================================================================\n\n');

%% Step 1: 元データの読み込み
fprintf('Step 1: Loading Original Data\n');

data_path = '../../data/tiredata/';
drivebrake_file = [data_path 'DriveBrake-Hoosier18.0×6.0-10-R20.csv'];
cornering_file = [data_path 'Cornering-Hoosier18.0×6.0-10-R20.csv'];

% DriveBrakeデータ
opts = detectImportOptions(drivebrake_file);
opts.DataLines = [4, Inf];  % ヘッダー行をスキップ
opts.VariableNames = {'ET','V','N','SA','IA','RL','RE','P','FX','FY','FZ','MX','MZ','NFX','NFY','RST','TSTI','TSTC','TSTO','AmbTmp','SR','SL'};
db_data = readtable(drivebrake_file, opts);
fprintf('  DriveBrake: %d samples\n', height(db_data));

% Corneringデータ
opts2 = detectImportOptions(cornering_file);
opts2.DataLines = [3, Inf];
opts2.VariableNames = {'ET','V','N','SA','IA','RL','RE','P','FX','FY','FZ','MX','MZ','NFX','NFY','RST','TSTI','TSTC','TSTO','AmbTmp','SR','CP'};
corn_data = readtable(cornering_file, opts2);
fprintf('  Cornering: %d samples\n', height(corn_data));

%% Step 2: データの統合と必要な列の抽出
fprintf('\nStep 2: Extracting Required Columns\n');

% 必要な列: SR, SA, Fz(正値化), IA, TSTC, Fx, Fy, Mx, Mz
% 注: FZは負の値として記録されているので正値化

db_extracted = table();
db_extracted.SR = db_data.SR;
db_extracted.SA = db_data.SA;
db_extracted.Fz = -db_data.FZ;  % 正値化
db_extracted.IA = db_data.IA;
db_extracted.TSTC = db_data.TSTC;
db_extracted.Fx = db_data.FX;
db_extracted.Fy = db_data.FY;
db_extracted.Mx = db_data.MX;
db_extracted.Mz = db_data.MZ;

corn_extracted = table();
corn_extracted.SR = corn_data.SR;
corn_extracted.SA = corn_data.SA;
corn_extracted.Fz = -corn_data.FZ;  % 正値化
corn_extracted.IA = corn_data.IA;
corn_extracted.TSTC = corn_data.TSTC;
corn_extracted.Fx = corn_data.FX;
corn_extracted.Fy = corn_data.FY;
corn_extracted.Mx = corn_data.MX;
corn_extracted.Mz = corn_data.MZ;

% 統合
all_data = [db_extracted; corn_extracted];
fprintf('  Combined: %d samples\n', height(all_data));

%% Step 3: 不正なデータの除去
fprintf('\nStep 3: Filtering Invalid Data\n');

% Fz が正で妥当な範囲
valid_idx = all_data.Fz > 50 & all_data.Fz < 3000;
% 異常値除去
valid_idx = valid_idx & abs(all_data.Fx) < 5000;
valid_idx = valid_idx & abs(all_data.Fy) < 5000;
valid_idx = valid_idx & abs(all_data.SR) < 0.5;
valid_idx = valid_idx & abs(all_data.SA) < 20;

filtered_data = all_data(valid_idx, :);
fprintf('  After filtering: %d samples (removed %d)\n', ...
        height(filtered_data), height(all_data) - height(filtered_data));

%% Step 4: ゼロスリップ点の追加
fprintf('\nStep 4: Adding Zero Slip Points\n');

% ゼロスリップ点（SR=0, SA=0でFx=Fy=0）を生成
Fz_values = [500, 750, 1000, 1250, 1500, 1750, 2000];
TSTC_values = [30, 40, 50, 60, 70, 80];
IA_values = [-3, -2, -1, 0, 1, 2, 3];

zero_slip_points = [];
for Fz = Fz_values
    for TSTC = TSTC_values
        for IA = IA_values
            % 転がり抵抗分の小さなFx（Fzに比例）
            Fx_rolling = -0.01 * Fz;  % 転がり抵抗係数 ≈ 0.01
            
            zero_slip_points = [zero_slip_points; ...
                0, 0, Fz, IA, TSTC, Fx_rolling, 0, 0, 0];
        end
    end
end

zero_slip_table = array2table(zero_slip_points, ...
    'VariableNames', {'SR', 'SA', 'Fz', 'IA', 'TSTC', 'Fx', 'Fy', 'Mx', 'Mz'});

fprintf('  Generated %d zero slip points\n', height(zero_slip_table));

%% Step 5: 対称性データの生成
fprintf('\nStep 5: Generating Symmetry-Enforced Data\n');

% 元データから対称点を生成
% Fy(SA, IA) = -Fy(-SA, -IA)
% Fx(SR) = -Fx(-SR)

symmetric_data = filtered_data;
symmetric_data.SR = -filtered_data.SR;
symmetric_data.SA = -filtered_data.SA;
symmetric_data.IA = -filtered_data.IA;
symmetric_data.Fx = -filtered_data.Fx;
symmetric_data.Fy = -filtered_data.Fy;
symmetric_data.Mx = -filtered_data.Mx;
symmetric_data.Mz = -filtered_data.Mz;

fprintf('  Generated %d symmetric points\n', height(symmetric_data));

%% Step 6: データの統合
fprintf('\nStep 6: Combining All Data\n');

% 元データ + 対称データ + ゼロスリップ点
augmented_data = [filtered_data; symmetric_data; zero_slip_table];

% シャッフル
rng(42);
idx = randperm(height(augmented_data));
augmented_data = augmented_data(idx, :);

fprintf('  Total augmented samples: %d\n', height(augmented_data));
fprintf('    Original: %d\n', height(filtered_data));
fprintf('    Symmetric: %d\n', height(symmetric_data));
fprintf('    Zero slip: %d\n', height(zero_slip_table));

%% Step 7: 統計の確認
fprintf('\nStep 7: Data Statistics\n');

fprintf('  SR:   mean=%.4f, std=%.4f, range=[%.3f, %.3f]\n', ...
        mean(augmented_data.SR), std(augmented_data.SR), ...
        min(augmented_data.SR), max(augmented_data.SR));
fprintf('  SA:   mean=%.4f, std=%.4f, range=[%.3f, %.3f]\n', ...
        mean(augmented_data.SA), std(augmented_data.SA), ...
        min(augmented_data.SA), max(augmented_data.SA));
fprintf('  Fz:   mean=%.1f, std=%.1f, range=[%.0f, %.0f]\n', ...
        mean(augmented_data.Fz), std(augmented_data.Fz), ...
        min(augmented_data.Fz), max(augmented_data.Fz));
fprintf('  Fx:   mean=%.1f, std=%.1f, range=[%.0f, %.0f]\n', ...
        mean(augmented_data.Fx), std(augmented_data.Fx), ...
        min(augmented_data.Fx), max(augmented_data.Fx));
fprintf('  Fy:   mean=%.1f, std=%.1f, range=[%.0f, %.0f]\n', ...
        mean(augmented_data.Fy), std(augmented_data.Fy), ...
        min(augmented_data.Fy), max(augmented_data.Fy));

% 理想的には mean(Fx) ≈ 0, mean(Fy) ≈ 0
if abs(mean(augmented_data.Fx)) < 50 && abs(mean(augmented_data.Fy)) < 50
    fprintf('\n  ✓ Data is now balanced (mean forces near zero)\n');
else
    fprintf('\n  ⚠ Data still has bias\n');
end

%% Step 8: 保存
fprintf('\nStep 8: Saving Augmented Data\n');

output_file = '../../data/tiredata/augmented_tire_data.csv';
writetable(augmented_data, output_file);
fprintf('  Saved to: %s\n', output_file);

% MATファイルも保存（高速読み込み用）
save('../../data/tiredata/augmented_tire_data.mat', 'augmented_data');
fprintf('  Saved to: augmented_tire_data.mat\n');

fprintf('\n=================================================================\n');
fprintf(' Augmentation Complete\n');
fprintf('=================================================================\n');
fprintf('\nNext: Run train_nn_with_augmented_data.m to retrain the model\n');
