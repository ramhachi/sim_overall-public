# `matlab/dl_tire_model/compareExperiments.m`

## 1. 概要

`compareExperiments` は、複数の実験ログ MAT ファイルを読み込み、設定値・学習情報・評価指標を表形式で比較し、必要ならテキストレポートとして保存する関数です。

## 2. ファイルの役割

- 実験 ID 群、または `all` 指定から比較対象ログを集めます。
- `loadExperimentLog` で各ログを読み込みます。
- `config`, `training_info`, `metrics` の代表値を抽出して比較テーブルを構築します。
- 最良実験を validation loss 最小で選び、結果を表示・保存します。

## 3. インターフェース

### 3.1 関数シグネチャ

```matlab
comparison_table = compareExperiments(experiment_ids, log_dir, output_file)
```

### 3.2 引数

- `experiment_ids`
  - 実験 ID のセル配列、または文字列 `'all'`
- `log_dir`（省略可）
  - ログディレクトリ。既定値は `'experiments/logs'`
- `output_file`（省略可）
  - 比較レポート保存先。空または未指定なら保存しません。

### 3.3 戻り値

- `comparison_table`
  - 以下の列を持つテーブルです。
    - `ExperimentID`
    - `LearningRate`
    - `BatchSize`
    - `MaxEpochs`
    - `Patience`
    - `ModelType`
    - `DropoutRate`
    - `TrainingTime_s`
    - `ValLoss`
    - `RMSE_Fx`
    - `RMSE_Fy`
    - `R2_Fx`
    - `R2_Fy`
    - `MeetsRequirements`

## 4. 依存関係

- プロジェクト関数: `loadExperimentLog`
- MATLAB 標準関数: `dir`, `fileparts`, `sortrows`, `disp`, `warning`, `fopen`, `fclose`, `fprintf`, `datetime`, `evalc`, `mkdir`, `fullfile`, `mat2str`

## 5. 処理の流れ

1. `log_dir` が未指定なら `'experiments/logs'` を使います。
2. `experiment_ids` が `'all'` の場合は `exp_*.mat` を列挙し、ファイル名から実験 ID を抽出します。
3. 各実験 ID について `loadExperimentLog` を呼び、読込失敗時は `warning` を出して空配列を格納します。
4. 空配列になった実験を取り除き、残ったログから比較列を個別ベクトルへ展開します。
5. それらを `table(...)` にまとめ、`ValLoss` 昇順で並べ替えます。
6. `val_loss_col` の最小値から最良実験を求め、その設定と性能を表示します。
7. `output_file` が指定されていれば、必要に応じて親ディレクトリを作成し、比較表と最良実験サマリーをテキスト出力します。

## 6. 関数・ローカル関数詳細

### 6.1 `compareExperiments`

- ログ構造体から参照しているフィールドは以下です。
  - `exp.config.learning_rate`, `batch_size`, `max_epochs`, `patience`, `model_type`, `dropout_rate`
  - `exp.training_info.TrainingTime`, `FinalValidationLoss`
  - `exp.metrics.Fx.RMSE`, `exp.metrics.Fy.RMSE`, `exp.metrics.Fx.R2`, `exp.metrics.Fy.R2`, `exp.metrics.meets_requirements`
- ファイル保存時のテーブル出力は `evalc('disp(comparison_table)')` を使って文字列化しています。
- ファイル書き込み中に例外が起きた場合でも `fclose(fid)` を呼んでから再送出します。

## 7. 注意点

- 最良実験の選定は `ValLoss` 最小だけで行い、`RMSE` や `R²` は補助表示です。
- `output_file` はテキストとして書かれるだけで、MAT や CSV は生成しません。
- 比較対象ログの構造が想定と異なると、フィールド参照で失敗します。
