# `tire_fitting/reporting/comparison_report.py`

## 役割

複数タイヤモデルの比較結果を Markdown レポートと PNG 図へまとめるモジュールです。  
指標表、選択理由、力-スリップ比較図、推定パラメータ一覧を 1 つの成果物へ統合します。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - `ComparisonReportData`
  - 出力ディレクトリ
  - レポート名
- 主な出力:
  - `<report_name>.md`
  - `plots/<report_name>_fx.png`
  - `plots/<report_name>_fy.png`
- 副作用:
  - 出力ディレクトリと `plots` ディレクトリ作成

## 主要クラス

### `ModelResult`
- 1 モデル・1 方向の `parameters` と `metrics` を持つ dataclass です。

### `ComparisonReportData`
- モデル結果、実験データ、選択済みモデル、選択理由、タイヤ名を束ねる入力 dataclass です。

## 主要関数

### `generate_comparison_report(report_data, output_dir, report_name='model_comparison_report')`
- 入力妥当性確認、図生成、Markdown 構築、ファイル保存をまとめて行うメイン API です。

### `_validate_report_data(report_data)`
- 実験データや選択結果が揃っているか確認します。

### `_generate_visualizations(...)`
- 縦力図・横力図の生成を方向別に呼び分けます。

### `_plot_longitudinal_comparison(...)` / `_plot_lateral_comparison(...)`
- 実測散布図に対し、各モデルの平均荷重での予測曲線を重ね描きします。

### `_create_model_instance(model_type, parameters)`
- `tmeasy` / `magic_formula` に応じてモデルインスタンスを生成します。

### `_build_report_content(...)`
- Markdown 全文を組み立てます。
- 内部で `_build_metrics_table()`, `_build_performance_analysis()`, `_build_parameters_section()` を使います。

## データフロー

1. `ModelFitter` と `ModelSelector` の結果を `ComparisonReportData` として受け取ります。
2. 実験データとモデル予測から図を作ります。
3. 指標表、性能差分析、パラメータ一覧を Markdown へ整形します。
4. 図とレポートを同じ出力ディレクトリへ保存します。

## システム内での位置づけ

hybrid パイプラインの **説明責任・可視化層** です。  
モデル選択結果を人が追える形へ落とし込み、`main_hybrid.py` の成果物を完結させます。
