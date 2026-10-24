# `tire_fitting/validate_predictions.py`

## 役割

保存済み TMeasy パラメータを使い、学習データ上のランダム点で予測精度を可視化する検証 CLI です。  
縦力と横力を別々に比較し、誤差統計と散布図群を出力します。

## 実行と入出力

- 実行例:
  - `python tire_fitting/validate_predictions.py --params output/tire_params/xxx.yaml`
- 主な入力:
  - DriveBrake / Cornering CSV
  - フィット済み YAML
  - 出力先
  - ランダムサンプル数
- 主な出力:
  - `validation_longitudinal.png`
  - `validation_lateral.png`
  - 標準出力への RMSE / MAE / 最大誤差サマリ

## 主要関数

### `load_fitted_parameters(yaml_path)`
- 旧形式 YAML から TMeasy パラメータを抽出します。

### `sample_random_points(df, n_samples=50, seed=42)`
- サンプル数を超える場合のみ無復元ランダム抽出します。

### `validate_longitudinal(drivebrake_path, params, output_dir, n_samples=50)`
- ロード → 前処理 → ランダム抽出 → `compute_longitudinal_force()` → 誤差統計 → 2x2 図作成の順です。

### `validate_lateral(cornering_path, params, output_dir, n_samples=50)`
- 横力版です。前処理は `FZ>=500` と `FY` 外れ値除去中心で、温度条件は使いません。

### `main()`
- 引数解析、出力先作成、パラメータ表示、縦横検証実行、サマリ表示を行います。

## データフロー

1. 既存 YAML から TMeasy パラメータを読みます。
2. TTC データを再読込し、簡易前処理後にランダム抽出します。
3. 予測値と実測値を比較し、誤差統計と可視化を作ります。

## システム内での位置づけ

本番フィッタではなく **事後検証用の補助スクリプト** です。  
生成済みパラメータの妥当性を視覚的に確認したいときに使います。
