# `tire_fitting/main_hybrid.py`

## 役割

TMeasy と Magic Formula を比較し、縦横別に最適モデルを選んで成果物へ落とす hybrid CLI です。  
データ読込、複数モデル最適化、モデル選択、品質検証、比較レポート生成、YAML / C++ 出力を 1 本で実行します。

## 実行と入出力

- 実行例:
  - `python tire_fitting/main_hybrid.py --drivebrake <csv> --cornering <csv> --output <dir> --tire-name <name>`
  - `python tire_fitting/main_hybrid.py ... --model-long magic_formula --model-lat tmeasy`
- 主な入力:
  - TTC データ 2 系統
  - `--model-long`, `--model-lat`, `--config`
  - `--remove-outliers`
- 主な出力:
  - `comparison_reports/hybrid_model_comparison.md`
  - `comparison_reports/plots/*.png`
  - `tire_params/<tire>.yaml`
  - `tire_params/<tire>.hpp`
- 戻り値:
  - `main()` は終了コード `0/1`

## 主要関数

### `load_config_file(config_path)` / `merge_config_with_args(args)`
- モデル選好設定 YAML を読み、CLI 優先で引数へ反映します。
- 有効値は `tmeasy`, `magic_formula`, `auto` です。

### `parse_arguments()`
- 必須は入力 2 本、出力先、タイヤ名です。
- 任意で荷重、外れ値除去、モデル選択方式、設定ファイルを受けます。

### `main()`
- 7 段の本体処理を持ちます。
  1. `TireDataLoader` で TTC データ読込
  2. 欠損と `FZ<=0` のみを除く軽い前処理
  3. `ModelFitter.fit_multiple_models()` で必要モデルだけ最適化
  4. `ModelSelector` で `AUTO` / `MANUAL` / `HYBRID` 選択
  5. `validate_fitting_quality()` で品質閾値検証
  6. `generate_comparison_report()` で比較レポート作成
  7. `export_hybrid_config()` と `export_cpp_header()` で成果物出力

## データフロー

1. DriveBrake / Cornering を最小限フィルタで保持し、学習点数を確保します。
2. 要求された方向ごとに、必要なモデルだけ並列フィットします。
3. `rmse`, `tic`, `r2` をもとに縦力モデルと横力モデルを別々に決めます。
4. 選択結果を validation・comparison report・exporter へ共通供給します。
5. 最終的に YAML / C++ / Markdown レポートへ同じ選択内容を反映します。

## システム内での位置づけ

hybrid タイヤモデル系の **本命エントリポイント** です。  
`model_selector.py`、`optimizer.ModelFitter`、`exporters/`、`reporting/`、`validation.py` を束ねる上位オーケストレータになっています。
