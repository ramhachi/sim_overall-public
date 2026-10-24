# `examples/generate_comparison_report_example.py`

## 1. 役割

`tire_fitting.reporting.comparison_report` の使い方を示す **比較レポート生成サンプル** です。  
TMeasy と Magic Formula の両結果、さらにハイブリッド選択理由まで含めた疑似データを作り、Markdown レポートを出力します。

## 2. 入出力

- 実行例: `python3 examples/generate_comparison_report_example.py`
- 入力: スクリプト内で合成するモデル結果と実験データ
- 出力:
  - `output/comparison_reports/hybrid_model_comparison...`
  - 関連プロット群

## 3. 関数

### `create_example_report()`
- 役割: サンプル一式を組み立てて比較レポートを生成します。
- 主な流れ:
  1. `ModelResult` を `tmeasy` / `magic_formula` の `fx` / `fy` ごとに作成
  2. 擬似的な縦力・横力実験データを `numpy` で生成
  3. `ComparisonReportData` にモデル選択結果と選択理由を詰める
  4. `generate_comparison_report()` でレポート保存
  5. 出力先と主要知見を表示
- 戻り値: 生成されたレポートファイルパス
- 副作用: `output/comparison_reports` 以下へファイルを書きます。

## 4. トップレベルフロー

- `__main__` では `create_example_report()` を実行します。
- 例外時は traceback を表示し、終了コード 1 で終了します。

## 5. サンプルとしての意味

- 「FX は Magic Formula が有利、FY は TMeasy が有利」という研究結果をそのまま例題にしています。
- 実データ読込例ではなく、API の呼び方を確認するための自己完結サンプルです。
