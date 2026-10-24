# `tire_fitting/validation.py`

## 役割

hybrid フィット結果が品質閾値を満たすか判定する検証モジュールです。  
RMSE / TIC / R² を方向別に評価し、集約済みレポートオブジェクトとテキストレポートを返します。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - `{'fx': {'model_type', 'metrics'}, 'fy': {...}}`
  - `ValidationThresholds`
- 主な出力:
  - `ValidationReport`
  - `generate_validation_report_text()` による整形済み文字列
- 例外:
  - `ValueError`: 閾値や比較指定が不正

## 主要クラス

### `ValidationStatus`
- `PASS`, `FAIL`, `WARNING` の列挙です。

### `ValidationThresholds`
- `rmse_max`, `tic_max`, `r2_min` を保持します。
- `__post_init__()` で範囲妥当性を検査します。

### `MetricValidation`
- 単一指標の実測値、閾値、判定状態、説明文を持ちます。

### `DirectionValidation`
- `fx` または `fy` の 3 指標判定をまとめます。
- `passed` プロパティで総合 PASS を返します。

### `ValidationReport`
- `fx_validation`, `fy_validation`, 閾値、総合状態、要約文を保持します。
- `get_failed_metrics()` で失敗項目だけ列挙できます。

## 主要関数

### `validate_metric(metric_name, actual_value, threshold_value, comparison='less_than')`
- 単一比較を PASS / FAIL メッセージ付きで返します。

### `validate_direction(direction, model_type, metrics, thresholds)`
- 方向ごとに RMSE / TIC / R² を検証し、`DirectionValidation` を返します。

### `validate_fitting_quality(fitting_results, thresholds=None)`
- `fx` と `fy` を順に検証し、ログ出力込みで `ValidationReport` を返します。

### `generate_validation_report_text(report)`
- レポートを人間向けテキストに変換します。

## データフロー

1. 上流の選択済みモデルと metric を受け取ります。
2. 指標ごとに閾値比較します。
3. 方向別結果を集約して総合判定へ変換します。
4. validation warning やレポート文字列を後段へ渡します。

## システム内での位置づけ

`main_hybrid.py` の **品質ゲート** です。  
選択済みモデルの採用可否を明示し、比較レポートやログに同じ判定基準を供給します。
