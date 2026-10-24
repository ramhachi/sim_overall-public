# `matlab/dl_tire_model/runHyperparameterSearch.m`

## 1. 概要

複数のハイパーパラメータ候補を試行し、検証損失にもとづいて最良設定を選定する関数です。グリッドサーチとランダムサーチの両方に対応し、各試行の学習・評価・ログ保存・モデル保存までを一括で実行します。

## 2. ファイルの役割

- ハイパーパラメータ探索の実行本体です。
- `search_space` から評価対象の設定群を生成します。
- 各設定についてモデル作成、学習、評価、実験ログ保存、モデル保存を順に実行します。
- `FinalValidationLoss` が最小の設定を `best_config` として返します。
- 探索途中で失敗した設定も `all_results` にエラー情報付きで残します。

## 3. インターフェース

- 形式: `[best_config, all_results] = runHyperparameterSearch(X_train, Y_train, X_val, Y_val, norm_params, search_space, search_type, num_trials)`
- 引数:
  - `X_train`: 学習入力行列
  - `Y_train`: 学習教師行列
  - `X_val`: 検証入力行列
  - `Y_val`: 検証教師行列
  - `norm_params`: 評価時に使う正規化パラメータ構造体
  - `search_space`: 試行候補を定義する構造体。各フィールドは候補値のセル配列
  - `search_type`: `'grid'` または `'random'`
  - `num_trials`: ランダムサーチ時の試行数。グリッドサーチでは未使用、未指定時は `10`
- 戻り値:
  - `best_config`: 最良の検証損失を与えた設定構造体
  - `all_results`: 各試行結果を格納したセル配列
    - 成功時の要素: `config`, `metrics`, `training_info`, `experiment_id`, `val_loss`
    - 失敗時の要素: `config`, `error`, `val_loss`

## 4. 依存関係

- 同ディレクトリ内の関数:
  - `createModelFromConfig`
  - `trainTireModel`
  - `evaluateModel`
  - `logExperiment`
  - `saveExperimentModel`
  - `createHyperparameterConfig`
- MATLAB 組み込み:
  - `fprintf`
  - `strcmpi`
  - `nargin`
  - `error`
  - `exist`
  - `mkdir`
  - `length`
  - `cell`
  - `struct`
  - `sum`
  - `cellfun`
  - `isfield`
  - `fieldnames`
  - `ndgrid`
  - `arrayfun`
  - `prod`
  - `randi`

## 5. 処理の流れ

1. 開始メッセージと探索方式を表示します。
2. `search_type` を判定し、以下のいずれかで設定群 `configs` を作ります。
   - `'grid'`: `generateGridSearchConfigs(search_space)`
   - `'random'`: `generateRandomSearchConfigs(search_space, num_trials)`
3. 全試行結果 `all_results`、最良損失 `best_val_loss`、最良設定 `best_config`、最良試行番号 `best_idx` を初期化します。
4. `experiments/logs` ディレクトリが無ければ作成します。
5. 各 `config` についてループし、順に以下を実行します。
   1. `createModelFromConfig(config)` でネットワーク作成
   2. `trainTireModel(...)` で学習
   3. `evaluateModel(...)` で検証
   4. `logExperiment(...)` で実験ログ保存
   5. `saveExperimentModel(...)` で `experiments/models` へモデル保存
   6. 結果を構造体にまとめて `all_results{i}` に格納
   7. `training_info.FinalValidationLoss` が最良なら `best_*` を更新
6. 試行中に例外が起きた場合はその試行をスキップし、`error` と `val_loss = inf` を結果へ残します。
7. 最後に総試行数、成功数、最良試行番号、最良設定の主要パラメータを表示します。

## 6. 関数・ローカル関数詳細

### `runHyperparameterSearch(X_train, Y_train, X_val, Y_val, norm_params, search_space, search_type, num_trials)`

- 本ファイルの公開関数です。
- 探索方式に応じた設定生成から、学習・評価・保存・最良設定選定までを管理します。
- 最良判定には `training_info.FinalValidationLoss` を使い、`metrics` ではなく学習器の最終検証損失を比較しています。

### `generateGridSearchConfigs(search_space)`

- `search_space` の全組み合わせを列挙して `configs` を返すローカル関数です。
- `fieldnames(search_space)` で探索対象パラメータ名を取得し、各候補値配列を `field_values` に集約します。
- `ndgrid` で各候補のインデックス組み合わせを作り、全組み合わせ数 `n_combinations` 回ループして設定構造体を生成します。
- 各設定は `createHyperparameterConfig()` の既定値を土台にし、探索対象フィールドだけ上書きします。

### `generateRandomSearchConfigs(search_space, num_trials)`

- ランダムサーチ用の設定群を作るローカル関数です。
- 各試行ごとに `createHyperparameterConfig()` を呼び、探索対象フィールドごとに `randi` で候補の添字を 1 つ選びます。
- 候補値は `search_space.(field_name)` のセル配列から取り出し、生成した設定を `configs{i}` に格納します。

## 7. 注意点

- `best_config` の更新条件は `FinalValidationLoss` のみで、他の指標は比較に使いません。
- 全試行が失敗すると `best_idx` は `0` のままで、末尾の要約表示で `all_results{best_idx}` を参照する処理が破綻する可能性があります。
- `saveExperimentModel` 呼び出し前に `experiments/models` をこの関数では作成していないため、保存先ディレクトリ作成は呼び出し先実装に依存します。
- グリッドサーチ用 `ndgrid` 呼び出しは可変個の探索次元を扱おうとしていますが、引数構築が特殊なため、探索次元数によっては MATLAB の呼び出し仕様に注意が必要です。
