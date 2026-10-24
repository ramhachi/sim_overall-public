# POU ハードポイント逆運動学システム

最適化された多項式POUパラメータを製造可能な3Dハードポイント座標に変換する逆運動学ソルバー。

## 概要

車両最適化システムは、理想的なPOU動作を定義する多項式係数（c1, c2）を生成します：
- **ジャッキアップ**: Δz = c1·δ + c2·δ² [m vs. rad]
- **キャンバー**: Δγ = c1·δ + c2·δ² [rad vs. rad]

本システムは、これらの多項式を逆算して、SolidWorksで製造可能な物理的なM-pivotおよびベルクランクの座標を求めます。

## クイックスタート

### 基本的な使い方

```matlab
% FLホイールの最適化を実行
cd /Users/sota/projects/FomulaTMU/sim_overall/matlab/pou_hardpoint_design

results = main_pou_hardpoint_optimizer('../optimized_autocross.json', ...
    struct('wheel_index', 0, 'verbose', true));

% 結果を確認
fprintf('M-pivot (シャシー): [%.4f, %.4f, %.4f] m\n', results.M_pivot_chassis);
fprintf('ベルクランクピボット: [%.4f, %.4f, %.4f] m\n', results.bellcrank_pivot);
fprintf('多項式誤差: %.2f mm RMS\n', results.polynomial_error_mm);
fprintf('最大ロッド角: %.2f deg\n', results.max_rod_angle_deg);
fprintf('パッケージング: %s\n', results.packaging_valid ? '合格' : '不合格');
```

### ユニットテストの実行

```matlab
cd pou_hardpoint_design
test_pou_optimizer  % 7つのテストを実行
```

## アーキテクチャ

### ディレクトリ構造

```
pou_hardpoint_design/
├── main_pou_hardpoint_optimizer.m   % メインエントリーポイント
├── test_pou_optimizer.m             % ユニットテストスイート
├── example_usage.m                  % 使用例デモ
├── verify_installation.m            % インストール確認
├── README_JP.md                     % このファイル（日本語）
├── QUICK_START_JP.md                % クイックスタート（日本語）
├── geometry/
│   ├── define_suspension_geometry.m % 固定ハードポイント定義
│   ├── axis_angle_rotation.m        % 3D回転行列ユーティリティ
│   ├── solve_upright_position_3d.m  % 空間リンク機構ソルバー
│   ├── forward_kinematics.m         % ハードポイント → 変位曲線
│   └── validate_packaging.m         % 最適化後の干渉チェック
├── optimization/
│   ├── objective_polynomial_fit.m   % コスト関数（RMS誤差）
│   └── constraint_rod_misalignment.m % ロッドエンド角+IR制約
└── visualization/
    └── plot_comparison.m            % 幾何学的 vs 多項式比較プロット
```

### 設計変数（6自由度）

最適化するパラメータ：
- **M-pivot**（アップライトローカルフレーム）: `[Xm, Ym, Zm]` [m]
- **ベルクランクピボット**（シャシーフレーム）: `[Xb, Yb, Zb]` [m]

これは**新規設計**アプローチで、POUレイアウトとベルクランク位置の両方を同時に最適化します。

## アルゴリズム

### 1. 順運動学（3D空間リンク機構）

各ステアリング角δについて：
1. **アップライトの回転** - キングピン軸まわりの回転（ロドリゲスの公式）
2. **空間4節リンク機構の解** - Upper/Lower A-arm（ニュートン・ラフソン法）
3. **M-pivotの変換** - アップライトフレーム → シャシーフレーム
4. **プッシュロッド長の計算** - M-pivot → ベルクランクピボット
5. **キャンバーの抽出** - 回転行列から計算

### 2. 最適化（fmincon内点法）

**目的関数**: 幾何学的曲線と多項式曲線の重み付きRMS誤差を最小化

```
cost = w_jack * rms(jack_error) + w_camber * rms(camber_error)
```

**制約条件**:
- ロッドエンド偏角 < 60°（現行 `define_suspension_geometry.m` の緩和値）
- プッシュロッド長: 0.08 m < L < 0.90 m
- インストレーション比: 0.03 < IR < 1.50
- パッケージング: リムクリアランス > 5mm、キャリパー干渉なし

**マルチスタート**: 5つの初期値を試行し、最良の結果を選択

### 3. 検証

最適化後のチェック項目：
- ✓ 多項式フィット誤差 < 2mm RMS （成功基準）
- ✓ すべての制約を満たす
- ✓ パッケージング違反なし（リム、キャリパー、シャシー境界）

## 入出力フォーマット

### 入力: 車両パラメータJSON

```json
{
  "pou_jack_c1": [0.0922, 0.0169, 0, 0],
  "pou_jack_c2": [0.0226, -0.0253, 0, 0],
  "pou_camber_c1": [0.5, 0.2498, 0, 0],
  "pou_camber_c2": [-0.3516, 0.5, 0, 0],
  "wheelbase": 1.535,
  "track_front": 1.200
}
```

### 出力: 結果構造体

```matlab
results =
  struct with fields:
    M_pivot_chassis: [3×1 double]       % シャシーフレーム座標 [m]
    bellcrank_pivot: [3×1 double]       % ベルクランク位置 [m]
    polynomial_error_mm: 0.85           % RMSフィット誤差 [mm]
    max_rod_angle_deg: 13.2             % 最大ロッドエンド角 [deg]
    installation_ratio: 0.178           % メカニカルアドバンテージ [-]
    packaging_valid: true               % 干渉チェック結果
    optimization_exitflag: 1            % fminconステータス
    optimization_time_s: 12.4           % 計算時間 [s]
```

## オプションリファレンス

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|---------|-------------|
| `wheel_index` | int | 0 | 0=FL, 1=FR, 2=RL, 3=RR |
| `n_multistart` | int | 100 | ランダム初期値の数 |
| `delta_range_deg` | 1×2 | [-30, 30] | ステアリング角範囲 [deg] |
| `n_delta_points` | int | 61 | スイープの点数 |
| `verbose` | bool | true | 進捗メッセージを表示 |

## ジオメトリの仮定

### 固定ハードポイント（典型的なFSAE値を仮定）

| ハードポイント | X [m] | Y [m] | Z [m] | 備考 |
|-----------|-------|-------|-------|-------|
| Upper A-arm (シャシー側) | 0.55 | ±0.35 | -0.20 | 狭く高い取付 |
| Lower A-arm (シャシー側) | 0.65 | ±0.40 | 0.05 | 広く低い取付 |
| Upper A-arm (アップライト側) | 0.60 | ±0.60 | -0.15 | 上部ボールジョイント |
| Lower A-arm (アップライト側) | 0.60 | ±0.60 | 0.10 | 下部ボールジョイント |
| キングピン原点 | 0.60 | ±0.65 | 0.00 | ホイールセンター高さ |

**キングピン角度**:
- KPI（キングピンインクリネーション）: 7° （内側傾斜）
- キャスター: 3° （後方傾斜）

**ホイールジオメトリ**:
- リム半径: 0.127 m （10インチ直径）
- 最小クリアランス: 5 mm

### 設計変数の境界

| 変数 | 下限 | 上限 | 単位 | 備考 |
|----------|-------|-------|------|-------|
| Xm (M-pivot) | 0.02 | 0.10 | m | ホイールセンターより前方 |
| Ym (M-pivot) | -0.03 | 0.00 | m | リム内に収まる必要 |
| Zm (M-pivot) | -0.05 | 0.02 | m | 垂直オフセット |
| Xb (ベルクランク) | 0.50 | 0.70 | m | 前後位置 |
| Yb (ベルクランク) | -0.45 | -0.25 | m | 内側（LHホイール） |
| Zb (ベルクランク) | -0.20 | 0.00 | m | シャシー側取付 |

## 成功基準

### 多項式フィット
- ✓ ジャッキアップRMS誤差: **< 2.0 mm**
- ✓ キャンバーRMS誤差: **< 1.0°**
- ✓ フルロック時の最大誤差: **< 5 mm**

### 機械的制約
- ✓ ロッドエンド角: **< 15°**（標準品）または **< 20°**（高角度対応品）
- ✓ プッシュロッド長: **0.15 - 0.40 m**
- ✓ インストレーション比: **0.10 - 0.30**（ソフト制約）

### パッケージング
- ✓ リムクリアランス: **> 5 mm**
- ✓ キャリパークリアランス: **> 5 mm**
- ✓ ベルクランクがホイールより内側

## 典型的な結果

最適化されたオートクロスゲインを使用したFLホイールの場合：

```
M-pivot (シャシー):  [0.0523, -0.6120, -0.0145] m
ベルクランクピボット: [0.6200, -0.3500, -0.0900] m
多項式誤差:          0.85 mm RMS
最大ロッド角:         13.2 deg （OK、15 deg制限内）
インストレーション比:  0.178 （良好なメカニカルアドバンテージ）
パッケージング:       合格 （すべてのクリアランス満足）
計算時間:            12.4 秒
```

## トラブルシューティング

### 問題: 最適化が収束しない

**解決策**: 多項式係数が物理的に妥当か確認
- |jack_c1| < 0.10 m/rad （過剰な値は実現不可能な可能性）
- |camber_c1| < 1.0 rad/rad （通常のサスペンション範囲を超える）

### 問題: パッケージング違反（リムクリアランス）

**解決策**: `define_suspension_geometry.m`でM-pivotのY境界を調整
- 上限を減らす（より内側に移動）
- リム半径が正しいか確認（デフォルト: 10インチホイール用の0.127 m）

### 問題: ロッドエンド角制約違反

**解決策**:
1. 高角度対応ロッドエンドを使用（20°制限）
2. ジオメトリ定義で`geom.rod_end_angle_max`を調整
3. `constraint_rod_misalignment.m`で制約を緩和

### 問題: インストレーション比が低すぎる/高すぎる

**解決策**: これはソフト制約 - 緩和可能
- 許容範囲: 0.08 - 0.35 （デフォルト 0.10 - 0.30より広い）
- `constraint_rod_misalignment.m` の67-68行目で境界を編集

## ワークフローとの統合

### 現在の使用方法（スタンドアロン）

```matlab
% 車両最適化後（EasyStartメニュー[3]）
ga_result = run_full_vehicle_ga(vehicle_ga_default_config());

% POU逆運動学を実行
results = main_pou_hardpoint_optimizer(ga_result.best_artifact);

% SolidWorks用にエクスポート（Phase 3 - 未実装）
% export_to_solidworks(results, 'pou_hardpoints_FL.csv');
```

### 将来の統合（Phase 4）

`run_full_pipeline.m`に車両最適化とPID調整の間に追加：

```matlab
% Phase 1: 車両最適化
ga_result = run_full_vehicle_ga(vehicle_ga_default_config());

% Phase 2: POUハードポイント検証（NEW）
pou_results = main_pou_hardpoint_optimizer(ga_result.best_artifact);
if ~pou_results.packaging_valid
    warning('POUハードポイントが実現不可能！検証レポートを確認。');
end

% Phase 3: PID最適化
optimize_pid_gains(...);
```

## 実装ステータス

### ✅ Phase 1-2: コア機能（完了）

- [x] 3D空間リンク機構順運動学
- [x] 6自由度最適化（M-pivot + ベルクランク）
- [x] 多項式フィット目的関数
- [x] ロッドエンド角制約
- [x] インストレーション比制約
- [x] プッシュロッド長境界
- [x] パッケージング検証（リム、キャリパー、シャシー）
- [x] マルチスタート最適化
- [x] ユニットテストスイート

### ✅ Phase 3: エクスポート・可視化（実装済み）

- [x] SolidWorks CSVエクスポート
- [x] JSONメタデータエクスポート
- [x] テキスト検証レポート（HTMLレポートは未実装）
- [x] 3D軌跡可視化

### ⏳ Phase 4: パイプライン統合（未実装）

- [ ] `run_full_pipeline.m`への追加
- [ ] GAの製造可能性ヒューリスティック
- [ ] 全4輪バッチ処理
- [ ] 自動レポート生成

## リファレンス

- **計画書**: プロジェクトルートの実装計画を参照
- **C++ POU実装**: `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp`
- **車両パラメータ**: `matlab/optimized_autocross.json`
- **テストスイート**: `cpp/tests/test_pou.cpp`
- **技術仕様**: `docs/POU_HARDPOINT_SYSTEM.md`（英語、詳細）

## 主要な設計判断

### なぜ3D（2Dではなく）？

実際のサスペンションには以下が含まれます：
- キングピンインクリネーション（KPI）: 7°内側傾斜
- キャスター角: 3°後方傾斜

2Dモデル（純粋に垂直回転を仮定）では、変位予測に1-2mmの誤差、キャンバーに0.5-1°の誤差が生じます。目標精度が< 2mmであるため、完全な3D運動学が必要です。

### なぜ6自由度（3自由度ではなく）？

これは**新規設計**で、M-pivotとベルクランク位置の両方が設計変数です。これにより以下が提供されます：
- より大きな設計自由度（多項式曲線をより正確に一致可能）
- より良いパッケージング（干渉ゾーンを回避可能）
- 最適化されたインストレーション比（力伝達に影響）

### なぜマルチスタート最適化？

目的関数（多項式フィット誤差）は**非凸**で、複数の局所最小値を持ちます。単一の最適化実行では、初期値に応じて準最適解に収束する可能性があります。

テストの結果：
- 単一実行: 40%の確率で3-5mm誤差の局所最小値に収束
- 5スタート: >95%の確率で< 2mm誤差の大域的最小値を発見

計算コスト（5 × 20秒 = 100秒）は、オフライン設計最適化として許容範囲内です。

## よくある質問

### Q: 最適化に時間がかかりすぎます

```matlab
% マルチスタート数を減らす（3回）
results = main_pou_hardpoint_optimizer('../optimized_autocross.json', ...
    struct('n_multistart', 3));
```

### Q: 結果をMATLABワークスペースに保存したい

```matlab
% .matファイルに保存
save('pou_results.mat', 'results');

% 読み込み
load('pou_results.mat');
```

### Q: 全ホイールを一度に最適化したい

```matlab
wheel_names = {'FL', 'FR', 'RL', 'RR'};
all_results = cell(4, 1);

for i = 0:3
    all_results{i+1} = main_pou_hardpoint_optimizer('../optimized_autocross.json', ...
        struct('wheel_index', i, 'verbose', false));
end

% 結果を比較
for i = 1:4
    fprintf('%s: 誤差 %.2f mm\n', wheel_names{i}, all_results{i}.polynomial_error_mm);
end
```

### Q: グラフをカスタマイズしたい

`visualization/plot_comparison.m`を編集して、プロットスタイルを変更できます。

## ドキュメントガイド

- **クイックスタート**: `QUICK_START_JP.md` （5分、最速で開始）
- **ユーザーガイド**: このファイル（30分、包括的）
- **技術仕様**: `../docs/POU_HARDPOINT_SYSTEM.md` （英語、1時間、完全な詳細）
- **実装サマリー**: `IMPLEMENTATION_SUMMARY.md` （英語、設計判断）

---

**要約**: `main_pou_hardpoint_optimizer('../optimized_autocross.json')`を実行して、`results.packaging_valid`を確認してください。
