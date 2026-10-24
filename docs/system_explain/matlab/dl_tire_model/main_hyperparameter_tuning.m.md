# `matlab/dl_tire_model/main_hyperparameter_tuning.m`

## 1. 概要

`main_hyperparameter_tuning.m` は、データ前処理から探索実行、比較レポート生成、最良設定での再学習、可視化、サマリ保存までをまとめたハイパーパラメータ探索用スクリプトです。

## 2. ファイルの役割

- 探索方式と探索空間を設定する
- `runHyperparameterSearch` を呼んで複数実験を実行する
- 成功実験の比較レポートを作る
- 最良設定で最終モデルを再学習して保存する

## 3. インターフェース

### スクリプトの主要変数・入出力

- `data_paths`
  - 学習用 CSV パス配列。
- `search_type`
  - `'grid'` または `'random'`。
- `num_random_trials`
  - ランダム探索試行数。
- `output_dir`
  - 出力先。既定は `'experiments'`。
- `preprocess_config`
  - `split_ratio` だけを持つ前処理設定。
- `search_space`
  - 探索候補をセル配列で持つ構造体。
- 主要出力変数
  - `best_config`, `all_results`, `comparison_table`
  - `final_trained_net`, `final_metrics`
  - `final_model_path`, `viz_dir`, `summary_path`

## 4. 依存関係

- `preprocessData`
- `runHyperparameterSearch`
- `compareExperiments`
- `createModelFromConfig`
- `trainTireModel`
- `evaluateModel`
- `exportModel`
- `denormalizeData`
- `generateAllVisualizations`
- MATLAB 標準関数
  - `predict`, `load`, `fprintf`, `input`, `mkdir`, `fopen`, `fclose`

## 5. 処理の流れ

### セクション: CONFIGURATION

- ワークスペースを初期化し、データパス、探索方式、試行数、出力先を設定します。
- `output_dir` を必要に応じて作成します。

### セクション: STEP 1 LOAD AND PREPROCESS DATA

- `preprocess_config.split_ratio = 0.8` を作り、`preprocessData(data_paths, preprocess_config)` を実行します。

### セクション: STEP 2 DEFINE HYPERPARAMETER SEARCH SPACE

- `learning_rate`, `batch_size`, `max_epochs`, `patience`, `model_type`, `dropout_rate` の候補を `search_space` に定義します。
- `search_type='grid'` の場合は全組合せ数を計算し、50 超なら `input('Continue? (y/n): ', 's')` で確認します。

### セクション: STEP 3 RUN HYPERPARAMETER SEARCH

- ランダム探索なら `num_random_trials` 付きで `runHyperparameterSearch` を呼びます。
- そうでなければ試行数無しで呼びます。

### セクション: STEP 4 GENERATE COMPARISON REPORT

- `all_results` から `experiment_id` を抜き出し、空要素を除外します。
- 成功実験があれば `compareExperiments(experiment_ids, log_dir, report_file)` を実行します。

### セクション: STEP 5 TRAIN FINAL MODEL WITH BEST CONFIGURATION

- `createModelFromConfig(best_config)` で最終モデルを作ります。
- `trainTireModel` と `evaluateModel` を実行します。
- `best_model_<timestamp>.mat` として `exportModel` で保存します。
- 検証データの予測を逆正規化し、`generateAllVisualizations` で図を保存します。

### セクション: STEP 6 SAVE TUNING SUMMARY

- 探索条件、探索空間、最良設定、最終評価、出力ファイル一覧をテキストファイルへ書き出します。

### セクション: WORKFLOW COMPLETE

- 主要出力ファイルを表示し、`final_metrics.meets_requirements` に応じてメッセージを分岐します。
- 最後に検索空間カスタマイズ例をコメントで残しています。

## 6. 関数・ローカル関数詳細

### スクリプト内の関数定義

- このファイルに実ローカル関数は定義されていません。
- 最終セクションは検索空間変更例のコメントです。

## 7. 注意点

- `grid` で候補数が多い場合は対話入力が必要で、自動実行向きではありません。
- `timestamp`, `report_file`, `final_model_path`, `viz_dir`, `log_dir` は前段成功時に作られるため、失敗経路では後段サマリ処理で未定義になり得ます。
- `model_type` の候補は既定で `'feedforward'` のみです。
