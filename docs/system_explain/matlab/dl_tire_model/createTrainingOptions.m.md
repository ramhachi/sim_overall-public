# `matlab/dl_tire_model/createTrainingOptions.m`

## 1. 概要

`createTrainingOptions` は、設定構造体と検証データを受け取り、`trainNetwork` 用の `trainingOptions('adam', ...)` を組み立てる関数です。

## 2. ファイルの役割

- 学習率・エポック数・バッチサイズ・早期終了条件を設定します。
- 検証データと検証頻度を埋め込みます。
- 実行環境を `auto` にして、GPU があれば自動利用できる設定を返します。

## 3. インターフェース

### 3.1 関数シグネチャ

```matlab
options = createTrainingOptions(config, X_val, Y_val)
```

### 3.2 引数

- `config`
  - 使用する主フィールド
    - `max_epochs`
    - `batch_size`
    - `learning_rate`
    - `patience`
    - `val_frequency`
  - 未定義フィールドは関数内で既定値が補われます。
- `X_val`
  - 検証入力
- `Y_val`
  - 検証ターゲット

### 3.3 戻り値

- `options`
  - `trainingOptions` が返す学習設定オブジェクト

## 4. 依存関係

- Deep Learning Toolbox 関数: `trainingOptions`
- MATLAB 標準関数: `isfield`, `fprintf`

## 5. 処理の流れ

1. `config` に必要フィールドがあるか確認し、なければ既定値を入れます。
2. `trainingOptions('adam', ...)` を生成します。
3. 学習率スケジュールは `piecewise`、`LearnRateDropFactor = 0.5`、`LearnRateDropPeriod = 20` に固定します。
4. 検証データは `{X_val, Y_val}` をそのまま設定します。
5. `ValidationPatience` に `config.patience` を設定し、`Shuffle='every-epoch'`, `Verbose=true`, `Plots='training-progress'`, `ExecutionEnvironment='auto'` を指定します。
6. 設定内容をコンソールへ表示して返します。

## 6. 関数・ローカル関数詳細

### 6.1 `createTrainingOptions`

- 既定値は `max_epochs=200`, `batch_size=256`, `learning_rate=0.001`, `patience=10`, `val_frequency=50` です。
- GPU 利用判定そのものはせず、実行環境選択は `auto` に委ねます。

## 7. 注意点

- `Plots='training-progress'` を固定しているため、GUI 環境を前提とした設定です。
- `ValidationData` は数値行列前提で設定しており、LSTM 用のシーケンス形式には別対応が必要な可能性があります。
