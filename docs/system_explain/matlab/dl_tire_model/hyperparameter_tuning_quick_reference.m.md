# `matlab/dl_tire_model/hyperparameter_tuning_quick_reference.m`

## 1. 概要

`hyperparameter_tuning_quick_reference.m` は、ハイパーパラメータ探索関連 API の使い方を短いコード断片で並べた参照用スクリプトです。処理本体ではなく、関数呼び出し例の集約が目的です。

## 2. ファイルの役割

- 設定生成、実験記録、実験比較、モデル保存・読込の呼び出し例を示す
- `runHyperparameterSearch` を使う最小例を示す
- 学習から最良モデル利用までの例示フローを 1 ファイルにまとめる

## 3. インターフェース

### スクリプトの主要変数

- `config`
  - `createHyperparameterConfig` が返す設定構造体。
- `experiment_id`
  - `logExperiment` が返す実験 ID。
- `exp_log`
  - `loadExperimentLog` の戻り値。
- `search_space`
  - 探索対象パラメータを保持する構造体。
- `best_config`, `all_results`
  - `runHyperparameterSearch` の戻り値。
- `experiment_ids`
  - 比較対象 ID 一覧。
- `comparison_table`
  - `compareExperiments` の戻り値。
- `model_path`
  - `saveExperimentModel` の戻り値。
- `trained_net`, `norm_params`, `best_net`, `best_params`
  - モデル読込・推論例で使う変数。

### 入出力

- スクリプト自身の戻り値はありません。
- 各行はワークスペース変数やファイルの存在を前提に、関数呼び出し例を示します。

## 4. 依存関係

- `createHyperparameterConfig`
- `logExperiment`
- `loadExperimentLog`
- `runHyperparameterSearch`
- `compareExperiments`
- `saveExperimentModel`
- `loadExperimentModel`
- `loadTireData`
- `preprocessData`
- `predictTireForces`

## 5. 処理の流れ

### セクション 1: CREATE HYPERPARAMETER CONFIGURATION

- 既定設定生成と、個別値を上書きした設定生成の例を示します。

### セクション 2: LOG EXPERIMENT

- `config`, `metrics`, `training_info` を `logExperiment` に渡す例を示します。
- 既定ディレクトリと任意ディレクトリの両方を記載しています。

### セクション 3: LOAD EXPERIMENT LOG

- 実験 ID からログを再読込する例です。

### セクション 4: RUN GRID SEARCH

- `search_space` を定義し、`'grid'` で全組合せ探索する例です。

### セクション 5: RUN RANDOM SEARCH

- より広い探索空間を作り、`'random', 20` で 20 試行だけ行う例です。

### セクション 6: COMPARE EXPERIMENTS

- 特定 ID 群比較、`'all'` 比較、比較結果保存の例を並べています。

### セクション 7: SAVE EXPERIMENT MODEL

- 実験 ID に紐づくモデル保存の例です。

### セクション 8: LOAD EXPERIMENT MODEL

- 実験 ID からモデルを読み戻す例です。

### セクション 9: COMPLETE WORKFLOW EXAMPLE

- `loadTireData` → `preprocessData` → `runHyperparameterSearch` → `compareExperiments` → `loadExperimentModel` → `predictTireForces` という一連例です。

### セクション 10: TIPS AND BEST PRACTICES

- ランダム探索から始めること、比較結果を保存することなどのコメントを列挙しています。

## 6. 関数・ローカル関数詳細

### スクリプト内の関数定義

- このファイルには実関数・ローカル関数は定義されていません。
- すべて他ファイルの関数呼び出し例です。

## 7. 注意点

- `metrics`, `training_info`, `trained_net`, `X_train` など多くの変数は、このスクリプト内で生成していません。
- そのまま先頭から実行すると未定義変数で停止する箇所があります。
- 参照用スクリプトであり、単独で完結するワークフロー実装ではありません。
