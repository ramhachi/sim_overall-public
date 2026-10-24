# `matlab/dl_tire_model/final_system_validation.m`

## 1. 概要

MATLAB の DL タイヤモデル一式を最終検証する統合関数です。  
単体テスト、性質ベーステスト、学習、性能要求確認、最終レポート生成を順番に実行します。

## 2. ファイルの役割

- このディレクトリの検証機能をまとめて走らせる最上位入口です。
- 要件充足状況を `results` 構造体に集約し、`output/final_validation_report.txt` を生成します。

## 3. インターフェース

### 公開関数 `results = final_system_validation()`

- 入力
  - なし
- 出力
  - `results`: 主に以下を含む構造体
    - `timestamp`
    - `unit_tests`
    - `property_tests`
    - `training`
    - `performance`
    - `report_path`
    - `all_requirements_met`

## 4. 依存関係

- 独自関数
  - `normalizeData`, `denormalizeData`
  - `computeRMSE`, `computeR2`
  - `loadTireData`
  - `preprocessDataWrapper`
  - `createTireModel`
  - `trainTireModel`
  - `evaluateModel`
- MATLAB 標準機能
  - `datetime`, `evalc`, `evalin`, `isfile`, `load`, `save`, `tic/toc`, `randn`, `randi`, `fopen`, `fprintf`, `fclose`
- 同ファイル内ローカル関数
  - `run_unit_tests`
  - `run_property_tests`
  - `train_full_model`
  - `verify_performance_requirements`
  - `generate_final_report`
  - `test_normalization_property`
  - `test_rmse_property`
  - `test_r2_property`
  - `test_normalization_roundtrip_property`
  - `status_string`
  - `check_mark`

## 5. 処理の流れ

### セクション 1: メイン関数 `final_system_validation`

1. `results` を初期化し、各種ステータスを `true` で開始します。
2. `run_unit_tests()` を呼んで単体テスト結果を格納します。
3. `run_property_tests()` を呼んで数理的性質の確認結果を格納します。
4. `train_full_model()` でフルデータ学習を実行します。
5. `verify_performance_requirements()` で RMSE、R²、学習時間、推論時間を確認します。
6. `generate_final_report()` でテキストレポートを生成します。
7. 最後に各結果の AND を取り、`results.all_requirements_met` を確定させます。

### セクション 2: 単体テスト実行

- `run_unit_tests` は 10 個の `.m` テストファイル名を固定配列で持ちます。
- 存在しないファイルは `SKIP` として `passed = -1` で記録します。
- 存在するものは `evalc(test_name)` で実行し、例外発生時のみ `FAIL` にします。

### セクション 3: 性質ベーステスト

- `run_property_tests` は 4 つの property test を順番に実行します。
- 対象は正規化、RMSE、R²、正規化往復変換です。

### セクション 4: 学習

- `train_full_model` は `../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv` だけを読み込みます。
- `split_ratio = 0.8` で前処理し、`max_epochs = 200`, `batch_size = 256`, `learning_rate = 0.001`, `patience = 10`, `val_frequency = 50`, `random_seed = 42` を指定して学習します。
- 成功時は `evaluateModel` を実行し、`output/final_trained_model.mat` に保存します。

### セクション 5: 性能要求確認

- `verify_performance_requirements` は以下を確認します。
  - `Fx/Fy RMSE < 100`
  - `Fx/Fy/Mx/Mz の R² > 0.95`
  - 学習時間 `< 30` 分
  - `predict` の 1000 サンプル平均時間 `< 1 ms`

### セクション 6: レポート生成

- `generate_final_report` は unit/property/training/performance の要約をテキストに書き出します。
- 出力先は `output/final_validation_report.txt` です。

## 6. 関数・ローカル関数詳細

### `final_system_validation()`

- 全体を束ねる公開関数です。
- 途中結果の表示は `status_string` を使って `PASS/FAIL` 化します。

### `run_unit_tests()`

- 固定リストのテストファイルを 1 本ずつ実行して集計します。
- `evalin('base', 'clear test_*')` で前回テスト変数の影響を減らしてから走らせます。

### `run_property_tests()`

- 4 つの property test を try-catch で個別実行し、失敗しても残りを続行します。

### `train_full_model()`

- データ読込、前処理、モデル生成、学習、評価、保存までをまとめた helper です。
- 例外時は `training_results.error` にメッセージを入れ、`success = false` にします。

### `verify_performance_requirements(training_results)`

- 学習失敗時は即 return し、詳細確認を行いません。
- 推論時間計測では `randn(1000, 5)` を直接 `predict` に渡して所要時間だけを見ています。

### `generate_final_report(results)`

- `results` の各フィールドをテキストへ整形出力します。
- training が失敗した場合は metrics ではなく `results.training.error` を記録します。

### `test_normalization_property()`

- 100 回反復し、`normalizeData` 後の平均が 0、標準偏差が 1 に近いか確認します。
- ランダムデータは `randn * 10 + 5` です。

### `test_rmse_property()`

- 100 回反復し、`computeRMSE` の結果と手計算 `sqrt(mean((y_pred - y_actual).^2))` を比較します。

### `test_r2_property()`

- 100 回反復し、`computeR2` の結果と `1 - SS_res / SS_tot` を比較します。

### `test_normalization_roundtrip_property()`

- 100 回反復し、`normalizeData` → `denormalizeData` の往復で相対誤差が許容値以下かを確認します。

### `status_string(passed)`

- true なら `'PASS'`、false なら `'FAIL'` を返す表示用 helper です。

### `check_mark(condition)`

- true なら `'✓'`、false なら `'✗'` を返す表示用 helper です。

## 7. 注意点

- `generate_final_report(results)` は、メイン関数が `results.all_requirements_met` を最終計算する前に呼ばれます。
- `run_unit_tests` では skipped テストがあっても `all_passed` は false になりません。
- `train_full_model` の学習データは DriveBrake 1 ファイルのみで、Cornering データは読み込みません。

