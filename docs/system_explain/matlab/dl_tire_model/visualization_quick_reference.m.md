# `matlab/dl_tire_model/visualization_quick_reference.m`

## 1. 概要

`visualization_quick_reference.m` は、可視化関数群の典型的な使い方を例示するクイックリファレンス用スクリプトです。全可視化一括生成、個別プロット生成、カスタム引数、学習後評価への組み込み、バッチ処理、エラーハンドリングなどを順番に示します。

## 2. ファイルの役割

このファイルは、`generateAllVisualizations` と個別可視化関数群の呼び方をまとめた利用例集です。スクリプト内では具体的な関数呼び出し例を示し、最後にベストプラクティスと参照先を表示します。

## 3. インターフェース

### スクリプトとして前提になる変数

- `X_features`
  - `N×5` 入力特徴量 `[SR, SA, Fz, IA, TSTC]`
- `Y_actual`
  - `N×4` 実測値 `[Fx, Fy, Mx, Mz]`
- `Y_pred`
  - `N×4` 予測値 `[Fx, Fy, Mx, Mz]`
- `output_dir`
  - 出力先ディレクトリ

### 実際に使う主関数

- `generateAllVisualizations`
- `generateScatterPlots`
- `generateResidualPlots`
- `generateTimeSeriesPlots`
- `generateOperatingConditionPlots`
- `createTireModel`
- `trainTireModel`
- `preprocessData`
- `denormalizeData`
- `predict`

## 4. 依存関係

- プロジェクト内関数
  - 上記の可視化関数群、前処理関数、学習関数
- MATLAB 組み込み
  - `linspace`, `size`, `sprintf`, `load`, `dir`, `mkdir`, `copyfile`, `fprintf`, `try/catch`

## 5. 処理の流れ

### 5.1 Example 1: 全可視化を一括生成

1. `output_dir = 'output/visualizations'` を設定します。
2. `generateAllVisualizations(X_features, Y_actual, Y_pred, output_dir)` を呼びます。
3. コメントで、20 個のプロットが生成される想定を説明します。

### 5.2 Example 2: 個別プロット生成

- 散布図、残差図、時系列図、運転条件別図を個別関数で生成する例を並べます。

### 5.3 Example 3: カスタムパラメータ

1. `generateTimeSeriesPlots(..., 'MaxSamples', 500)` を示します。
2. `generateOperatingConditionPlots(..., 'NumBins', 10)` を示します。
3. `time_vector` を作り、`'TimeVector'` 指定の例を示します。

### 5.4 Example 4: モデル評価フローへの統合

1. `data_paths` と `config.split_ratio` を設定して `preprocessData` を呼びます。
2. `createTireModel` と `trainTireModel` でモデルを学習します。
3. `predict(trained_net, X_val)` で予測します。
4. `denormalizeData` で予測値、正解値、特徴量を物理量へ戻します。
5. `generateAllVisualizations` で評価用図を作成します。

### 5.5 Example 5: 複数モデルのバッチ処理

1. `models = {'model_v1', 'model_v2', 'model_v3'}` をループします。
2. 各 `.mat` を読み込みます。
3. 同じ `X_val` で予測し、各モデル専用ディレクトリへ図を出力します。

### 5.6 Example 6: エラーハンドリング

- `try/catch` で `generateAllVisualizations` を包み、次元や列数の前提をメッセージで案内します。

### 5.7 Example 7: 選択的可視化

- 目的別に散布図・残差図・時系列図・条件別図だけを作る例を再掲します。

### 5.8 Example 8: 生成ファイルの後処理

1. `dir(fullfile(output_dir, '*.png'))` で生成画像を列挙します。
2. `output/report_figures` を作成します。
3. `scatter_Fx.png` と `scatter_Fy.png` を `copyfile` でレポート用ディレクトリへコピーします。

### 5.9 Tips and Best Practices

- 正規化を戻してから可視化すること、一括生成関数と個別関数の使い分け、大規模データ向け引数、出力整理、定期的な図確認をコメントで整理しています。
- 最後に `help` 参照先を表示します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- 利用例を順に並べた説明用スクリプトで、ローカル関数は持ちません。
- 多くの例は、事前に `X_features`, `Y_actual`, `Y_pred`, `X_val` などが用意されている前提です。
- Example 4 だけは前処理から学習まで含む比較的完結したサンプルです。

## 7. 注意点

- スクリプト冒頭で `X_features`, `Y_actual`, `Y_pred` を「Assume you have」としており、Example 1〜3 はその変数が存在しないとそのままでは実行できません。
- Example 5 では `X_val`, `X_features`, `Y_actual` が既に利用可能である前提です。
- コメント中の「20 plot files」や各内訳は、このファイル自身では確認せず説明として記載されています。
