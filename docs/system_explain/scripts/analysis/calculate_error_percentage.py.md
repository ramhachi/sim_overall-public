# `scripts/analysis/calculate_error_percentage.py`

## 1. 役割

ハイブリッドタイヤモデルの予測誤差を、**力の絶対量に対する割合として再評価** する解析スクリプトです。  
FX は Magic Formula、FY は TMeasy を使う前提の固定パラメータで、MAPE や百分位誤差を出します。

## 2. 入出力

- 実行例: `python3 scripts/analysis/calculate_error_percentage.py`
- 入力:
  - `output/comparison_reports/hybrid_model_comparison.md` の存在確認
  - TTC 実験 CSV
  - スクリプト内固定パラメータ
- 出力: 標準出力の詳細誤差分析

## 3. 関数

### `calculate_error_statistics(actual, predicted, force_name)`
- 役割: 誤差率統計をまとめて計算します。
- 計算項目:
  - 絶対誤差
  - 相対誤差 %
  - MAPE
  - median APE
  - 90/95 パーセンタイル
  - 最大誤差 %
  - RMSE とその平均力比 %
  - ピーク力誤差 %
- 戻り値: 解析結果辞書

### `main()`
- 役割: CLI 入口です。
- 主な流れ:
  1. 比較レポート存在確認
  2. モデル種別とパラメータを固定定義
  3. `TireDataLoader` で DriveBrake/Cornering データ読込
  4. フィット時に近い前処理を実施
  5. `MagicFormulaModel.calculate_fx()` で FX 予測
  6. `TMeasyModel.calculate_fy()` で FY 予測
  7. それぞれ誤差統計を計算・表示
  8. まとめ表と総評を表示
- 戻り値: `0`

## 4. 注意点

- レポートから数値を自動抽出せず、実際にはスクリプト内へ固定値を書いています。
- `yaml` を import していますが、このファイルでは未使用です。
