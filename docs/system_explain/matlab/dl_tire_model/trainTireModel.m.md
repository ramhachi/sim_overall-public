# `matlab/dl_tire_model/trainTireModel.m`

## 1. 概要

`trainTireModel` は、与えられたネットワークと学習・検証データを使ってタイヤモデルを学習する関数です。GPU 利用可否の確認、学習オプション生成、学習発散検出、GPU メモリ不足時の再試行と CPU フォールバックを行います。

## 2. ファイルの役割

学習処理の実行本体として、`trainNetwork` 呼び出しを包み込み、典型的な失敗ケースに対する回復処理を提供します。特に mini-batch の縮小と CPU への切り替えは、この関数内で完結します。

## 3. インターフェース

### 関数シグネチャ

```matlab
[trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, config)
```

### 引数

- `net`
  - `createTireModel` などで作成したネットワーク構造。
- `X_train`
  - 訓練入力。コメントでは `N×5`。
- `Y_train`
  - 訓練出力。コメントでは `N×4`。
- `X_val`
  - 検証入力。コメントでは `M×5`。
- `Y_val`
  - 検証出力。コメントでは `M×4`。
- `config`
  - 学習設定構造体。コメント上の主な項目は以下です。
    - `max_epochs`
    - `batch_size`
    - `learning_rate`
    - `patience`
    - `val_frequency`
    - `random_seed`

### 戻り値

- `trained_net`
  - 学習済みネットワーク。
- `training_info`
  - `trainNetwork` の返却情報を基にした構造体。
  - この関数で `TrainingTime` が追加設定されます。

### 例外

- `training_info.TrainingLoss` に `NaN` が含まれる場合
  - `trainTireModel:TrainingDivergence`
- GPU メモリ不足以外の例外
  - そのまま再送出されます。

## 4. 依存関係

- プロジェクト内関数
  - `checkGPUAvailability`
  - `createTrainingOptions`
- Deep Learning Toolbox
  - `trainNetwork`
- MATLAB 組み込み
  - `fprintf`, `rng`, `tic`, `toc`, `isfield`, `contains`, `floor`, `rethrow`

## 5. 処理の流れ

1. `config` が未指定なら空構造体を作ります。
2. `config.random_seed` が無ければ `42` を設定し、`rng(config.random_seed)` で再現性を固定します。
3. `checkGPUAvailability()` を呼んで GPU 状態を確認します。
4. `createTrainingOptions(config, X_val, Y_val)` で学習オプションを生成します。
5. `tic` で計時を開始し、`trainNetwork(X_train, Y_train, net, options)` を実行します。
6. 学習後、`training_info.TrainingLoss` に `NaN` があれば学習発散として例外を投げます。
7. 正常終了時は `TrainingTime` を追加し、学習時間と最終検証損失を表示します。
8. 例外発生時、メッセージまたは識別子に `out of memory` / `OutOfMemory` が含まれれば GPU メモリ不足とみなします。
9. GPU メモリ不足時は `config.batch_size` を半減します。
10. 半減後の `batch_size < 16` なら CPU 実行へ切り替えます。
11. そうでなければ縮小 batch size で `createTrainingOptions` を再実行し、再学習します。
12. どの再試行でも失敗した場合は捕捉した例外を再送出します。

## 6. 関数・ローカル関数詳細

### `trainTireModel`

- 学習オプション作成と `trainNetwork` 実行を司る主関数です。
- GPU 可否の取得結果は `gpu_available`, `gpu_info` に受けますが、このファイル内では表示や分岐には使っていません。
- CPU フォールバック時は `options.ExecutionEnvironment = 'cpu'` を明示的に設定します。
- ローカル関数は定義されていません。

## 7. 注意点

- GPU メモリ不足回復では `config.batch_size` が存在する前提で半減しているため、既定値設定は `createTrainingOptions` 側に委ねられています。
- 発散判定は `TrainingLoss` の `NaN` のみで、極端な値や `Inf` は個別判定していません。
- CPU フォールバック時も同一 `net` と同一データを使うため、環境だけが変わります。
