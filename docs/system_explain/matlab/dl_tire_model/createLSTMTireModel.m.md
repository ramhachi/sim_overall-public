# `matlab/dl_tire_model/createLSTMTireModel.m`

## 1. 概要

`createLSTMTireModel` は、時系列入力を想定した 2 層 LSTM 回帰ネットワークを `layerGraph` として返す関数です。

## 2. ファイルの役割

- シーケンス入力から `Fx, Fy, Mx, Mz` を推定する LSTM モデルを定義します。
- `createModelFromConfig` から `model_type='lstm'` のときに利用されます。

## 3. インターフェース

### 3.1 関数シグネチャ

```matlab
net = createLSTMTireModel()
```

### 3.2 戻り値

- `net`
  - LSTM ベースの `layerGraph`

## 4. 依存関係

- Deep Learning Toolbox 関数
  - `sequenceInputLayer`
  - `lstmLayer`
  - `dropoutLayer`
  - `fullyConnectedLayer`
  - `regressionLayer`
  - `layerGraph`

## 5. 処理の流れ

1. `sequenceInputLayer(5)` を入力層として定義します。
2. `lstmLayer(64, 'OutputMode', 'sequence')` を置き、時系列を次段へ渡します。
3. `dropoutLayer(0.2)` を入れます。
4. `lstmLayer(32, 'OutputMode', 'last')` で最終時刻の特徴を取り出します。
5. 再度 `dropoutLayer(0.2)` を入れます。
6. `fullyConnectedLayer(4)` と `regressionLayer` を追加して `layerGraph` を返します。

## 6. 関数・ローカル関数詳細

### 6.1 `createLSTMTireModel`

- 入力特徴量数は 5、出力数は 4 に固定です。
- 第 1 LSTM は系列全体を出力し、第 2 LSTM は最後の時刻だけを出力します。

## 7. 注意点

- このファイル単体では系列長やシーケンス整形を扱いません。
- `preprocessDataWrapper` が返す通常の数値行列と組み合わせる場合、入力形式の整合は呼び出し側で確認が必要です。
