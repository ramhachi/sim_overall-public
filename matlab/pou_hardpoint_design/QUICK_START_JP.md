# POU ハードポイント最適化 - クイックスタートガイド

**5分でセットアップ・初回実行**

## 前提条件

✓ MATLAB R2020a以降（Optimization Toolbox が必要）
✓ 最適化済み車両パラメータJSONファイル（`optimized_autocross.json`）

## インストール

MATLABを起動後、以下を実行：

```matlab
% 1. プロジェクトルートに移動
cd /Users/sota/projects/FomulaTMU/sim_overall/matlab

% 2. パスを追加（自動）
addpath('pou_hardpoint_design');
addpath('pou_hardpoint_design/geometry');
addpath('pou_hardpoint_design/optimization');
addpath('pou_hardpoint_design/visualization');
```

または、`pou_hardpoint_design`ディレクトリ内で作業する場合：

```matlab
% pou_hardpoint_designディレクトリに移動
cd /Users/sota/projects/FomulaTMU/sim_overall/matlab/pou_hardpoint_design

% サブディレクトリのパスを追加
addpath('geometry');
addpath('optimization');
addpath('visualization');
```

## インストール確認

```matlab
% インストール確認スクリプトを実行
verify_installation
```

すべてのチェックが✓で通れば正常にインストールされています。

## JSONファイルの確認

最適化を実行する前に、JSONファイルが正しく読み込めるか確認：

```matlab
% JSON読み込みテストを実行
test_json_loading
```

このテストが成功すれば、`optimized_autocross.json`が正しい形式であることが確認できます。

## 基本的な使い方（3行で実行）

```matlab
% フロント左ホイールの最適化を実行
results = main_pou_hardpoint_optimizer('../optimized_autocross.json', ...
    struct('wheel_index', 0, 'verbose', true));
```

**注意**: `pou_hardpoint_design`ディレクトリ内で実行する場合は `../optimized_autocross.json` を使用。
`matlab`ディレクトリ内で実行する場合は `optimized_autocross.json` を使用。

## 結果の確認

```matlab
% サマリーを表示
fprintf('M-pivot座標:  [%.3f, %.3f, %.3f] m\n', results.M_pivot_chassis);
fprintf('誤差:         %.2f mm RMS\n', results.polynomial_error_mm);
fprintf('パッケージング: %s\n', results.packaging_valid ? '合格' : '不合格');

% グラフ表示
plot_comparison(results);
```

## 期待される出力

```
=== POU Hardpoint Inverse Kinematics Optimizer ===
Loading vehicle parameters from: ../optimized_autocross.json
Wheel: FL
Target Polynomial Coefficients:
  Jack-up:  c1=0.0922 m/rad, c2=0.0226 m/rad²
  Camber:   c1=0.5000 rad/rad, c2=-0.3516 rad/rad²

Running optimization with 5 initial guesses...
[fmincon出力...]

========== OPTIMIZATION RESULTS ==========
Exit Status: 1 (converged)
Solve Time: 15.2 seconds

--- Hardpoint Coordinates ---
M-pivot (chassis): [0.0523, -0.6120, -0.0145] m
Bellcrank pivot:   [0.6200, -0.3500, -0.0900] m

--- Performance Metrics ---
Polynomial Fit Error (RMS): 0.85 mm
Max Rod-End Angle:          13.2 deg (limit: 15 deg)
Installation Ratio:         0.178 (target: 0.15-0.25)

--- Packaging Validation ---
Status: ✓ PASS (all constraints satisfied)
==========================================
```

## オプション設定

| オプション | デフォルト | 説明 |
|-----------|----------|------|
| `wheel_index` | 0 | 0=FL, 1=FR, 2=RL, 3=RR |
| `n_multistart` | 5 | ランダム初期値の数（多いほど精度向上） |
| `verbose` | true | 進捗メッセージを表示 |
| `delta_range_deg` | [-30, 30] | ステアリング角の範囲 [deg] |
| `n_delta_points` | 61 | サンプル点数 |

## 詳細な実行例

### 例1: フロント左ホイール（デフォルト設定）

```matlab
% pou_hardpoint_designディレクトリ内で実行
cd /Users/sota/projects/FomulaTMU/sim_overall/matlab/pou_hardpoint_design

results_FL = main_pou_hardpoint_optimizer('../optimized_autocross.json', ...
    struct('wheel_index', 0));

% 結果を表示
plot_comparison(results_FL);
```

### 例2: フロント右ホイール

```matlab
results_FR = main_pou_hardpoint_optimizer('../optimized_autocross.json', ...
    struct('wheel_index', 1));
```

### 例3: 全4輪を順番に最適化

```matlab
wheel_names = {'FL', 'FR', 'RL', 'RR'};
all_results = cell(4, 1);

for i = 0:3
    fprintf('\n=== 最適化中: %s ホイール ===\n', wheel_names{i+1});

    all_results{i+1} = main_pou_hardpoint_optimizer('../optimized_autocross.json', ...
        struct('wheel_index', i, 'verbose', false));

    fprintf('完了: 誤差 %.2f mm, パッケージング %s\n', ...
        all_results{i+1}.polynomial_error_mm, ...
        all_results{i+1}.packaging_valid ? '合格' : '不合格');
end

% 結果サマリー
fprintf('\n=== 全ホイール結果サマリー ===\n');
for i = 1:4
    fprintf('%s: 誤差 %.2f mm, ロッドエンド角 %.1f deg, IR %.3f\n', ...
        wheel_names{i}, ...
        all_results{i}.polynomial_error_mm, ...
        all_results{i}.max_rod_angle_deg, ...
        all_results{i}.installation_ratio);
end
```

### 例4: カスタム設定（高速実行）

```matlab
% 計算時間を短縮（精度は若干低下）
options = struct();
options.wheel_index = 0;        % フロント左
options.n_multistart = 3;       % 3回の初期値（デフォルト5回より高速）
options.verbose = false;        % 詳細出力なし

results = main_pou_hardpoint_optimizer('../optimized_autocross.json', options);

% 結果にアクセス
M_pivot = results.M_pivot_chassis;      % [3×1] M-pivot座標 [m]
bellcrank = results.bellcrank_pivot;    % [3×1] ベルクランク座標 [m]
error_mm = results.polynomial_error_mm;  % RMS誤差 [mm]
rod_angle = results.max_rod_angle_deg;   % 最大ロッドエンド角 [deg]
IR = results.installation_ratio;         % インストレーション比 [-]
valid = results.packaging_valid;         % パッケージングチェック (bool)

% 結果を表示
fprintf('\n=== 最適化結果 ===\n');
fprintf('M-pivot (シャシーフレーム): [%.4f, %.4f, %.4f] m\n', M_pivot);
fprintf('ベルクランクピボット:       [%.4f, %.4f, %.4f] m\n', bellcrank);
fprintf('多項式フィット誤差:         %.2f mm RMS\n', error_mm);
fprintf('最大ロッドエンド角:         %.2f deg (制限: 15 deg)\n', rod_angle);
fprintf('インストレーション比:       %.3f (推奨: 0.15-0.25)\n', IR);
fprintf('パッケージング:             %s\n', valid ? '✓ 合格' : '✗ 不合格');
```

## テスト実行

### ユニットテスト（7つのテストを実行）

```matlab
cd /Users/sota/projects/FomulaTMU/sim_overall/matlab/pou_hardpoint_design
test_pou_optimizer
```

期待される出力：
```
=== POU Hardpoint Optimizer Test ===

Test 1: Suspension Geometry Definition
  ✓ Geometry defined successfully
  ...

Test 7: Packaging Validation
  ✓ Packaging validation PASSED

=== All Tests Completed Successfully ===
```

### フルワークフローのデモ

```matlab
cd /Users/sota/projects/FomulaTMU/sim_overall/matlab/pou_hardpoint_design
example_usage
```

## トラブルシューティング

### 問題: "Cannot open file: ../optimized_autocross.json"

**原因**: ファイルが見つからない、またはパスが間違っている

**解決策1**: フルパスを指定
```matlab
results = main_pou_hardpoint_optimizer(...
    '/Users/sota/projects/FomulaTMU/sim_overall/matlab/optimized_autocross.json', ...
    struct('wheel_index', 0));
```

**解決策2**: 現在のディレクトリを確認
```matlab
pwd  % 現在のディレクトリを表示

% pou_hardpoint_design内にいる場合
% → '../optimized_autocross.json' を使用

% matlab内にいる場合
% → 'optimized_autocross.json' を使用
```

### 問題: "Optimization failed to find feasible solution"

**原因**: 多項式係数が物理的に実現不可能

**確認方法**:
```matlab
% JSONファイルを読み込み
fid = fopen('../optimized_autocross.json', 'r');
json_text = fread(fid, '*char')';
fclose(fid);
params = jsondecode(json_text);

% FLホイールの係数を確認
fprintf('Jack c1: %.4f (推奨: < 0.10)\n', params.pou_jack_c1(1));
fprintf('Jack c2: %.4f (推奨: < 0.05)\n', params.pou_jack_c2(1));
fprintf('Camber c1: %.4f (推奨: < 1.0)\n', params.pou_camber_c1(1));
fprintf('Camber c2: %.4f (推奨: < 0.5)\n', params.pou_camber_c2(1));
```

**解決策**: 係数が極端に大きい場合は、車両最適化の制約を強化

### 問題: "Packaging violations" （パッケージング違反）

**確認方法**:
```matlab
if ~results.packaging_valid
    fprintf('パッケージング違反:\n');
    for i = 1:length(results.violations)
        fprintf('  %d. %s\n', i, results.violations{i});
    end
end
```

**解決策**:
1. `geometry/define_suspension_geometry.m` を編集
2. M-pivotの境界を調整（Yの上限をより内側に）
3. リム半径が正しいか確認（デフォルト: 0.127 m = 10インチ）

### 問題: "Maximum rod-end angle exceeded" （ロッドエンド角超過）

**原因**: ジャッキアップ係数が大きすぎる（c1 > 0.08 m/rad）

**解決策**:
```matlab
% 高角度対応ロッドエンドを使用する場合（20度制限）
% geometry/define_suspension_geometry.m の 87行目付近を編集:
% geom.rod_end_angle_max = deg2rad(20);  % 15°から20°に変更
```

## 次のステップ

1. ✅ 上記の基本例を実行
2. ✅ グラフを確認 (`plot_comparison(results)`)
3. ⚠️ 実際のCAD座標に更新
   - `geometry/define_suspension_geometry.m` を編集
   - A-armハードポイントをSolidWorksから測定して入力
4. ⚠️ 全4輪について実行 (`wheel_index` を変更)
5. ⏳ SolidWorks用にエクスポート（Phase 3 - 未実装）

## ジオメトリのカスタマイズ方法

実際の車両に適用する前に、サスペンションのハードポイント座標を更新する必要があります：

### 手順

1. **SolidWorksでハードポイント座標を測定**
   - Upper A-arm chassis-side ball joint (シャシー側上部ボールジョイント)
   - Lower A-arm chassis-side ball joint (シャシー側下部ボールジョイント)
   - Upper A-arm upright-side ball joint (アップライト側上部ボールジョイント)
   - Lower A-arm upright-side ball joint (アップライト側下部ボールジョイント)
   - Kingpin axis origin and direction (キングピン軸の原点と方向)

2. **`geometry/define_suspension_geometry.m` を編集**

```matlab
% 例: 実測値を入力（FLホイール、シャシーフレーム座標）
geom.upper_arm_chassis = [
    0.550;    % X: 前方 [m]
    -0.350;   % Y: 右方向（左輪は負） [m]
    -0.200    % Z: 下方向 [m]
];

geom.lower_arm_chassis = [
    0.650;    % X [m]
    -0.400;   % Y [m]
    0.050     % Z [m]
];

% キングピン軸（実測のKPI角とキャスター角を使用）
kpi_angle = deg2rad(7);      % 実測値に置き換え
caster_angle = deg2rad(3);   % 実測値に置き換え
```

3. **再度最適化を実行**

```matlab
results = main_pou_hardpoint_optimizer('../optimized_autocross.json');
```

## ドキュメント

- **クイックスタート**: このファイル（5分）
- **ユーザーガイド**: `README.md`（英語、30分、包括的）
- **技術仕様**: `../docs/POU_HARDPOINT_SYSTEM.md`（英語、1時間、詳細）
- **実装サマリー**: `IMPLEMENTATION_SUMMARY.md`（英語、設計判断）

## ヘルプ

```matlab
help main_pou_hardpoint_optimizer    % 関数ドキュメント
type test_pou_optimizer.m            % 使用例を確認
type example_usage.m                 % フルワークフロー
```

## よくある質問

**Q: 計算時間を短くしたい**
```matlab
% n_multistartを3に減らす（精度は若干低下）
results = main_pou_hardpoint_optimizer('../optimized_autocross.json', ...
    struct('n_multistart', 3));
```

**Q: 詳細な出力を見たい**
```matlab
% verbose = true で実行（デフォルト）
results = main_pou_hardpoint_optimizer('../optimized_autocross.json', ...
    struct('verbose', true));
```

**Q: 結果をファイルに保存したい**
```matlab
% MATファイルに保存
save('pou_results_FL.mat', 'results');

% 後で読み込み
load('pou_results_FL.mat');
```

**Q: グラフを画像ファイルとして保存したい**
```matlab
fig = plot_comparison(results);
saveas(fig, 'pou_optimization_FL.png');
print(fig, 'pou_optimization_FL.pdf', '-dpdf');
```

---

**要約**: `main_pou_hardpoint_optimizer('../optimized_autocross.json')` を実行して、`results.packaging_valid` を確認。
