# `matlab/dl_tire_model/createDeepTireModel.m`

## 1. 概要

`createDeepTireModel` は、5 入力・4 出力の深い全結合回帰ネットワークを `layerGraph` として返す関数です。

## 2. ファイルの役割

- 通常の `createTireModel` より深い 5 隠れ層モデルを定義します。
- タイヤ状態 5 変数から `Fx, Fy, Mx, Mz` を回帰するための構造を返します。

## 3. インターフェース

### 3.1 関数シグネチャ

```matlab
net = createDeepTireModel()
```

### 3.2 戻り値

- `net`
  - `layerGraph(layers)` の結果
  - `trainNetwork` に渡すネットワーク構造です。

## 4. 依存関係

- Deep Learning Toolbox 関数
  - `featureInputLayer`
  - `fullyConnectedLayer`
  - `reluLayer`
  - `dropoutLayer`
  - `regressionLayer`
  - `layerGraph`

## 5. 処理の流れ

1. 入力層として `featureInputLayer(5, 'Normalization', 'none')` を定義します。
2. 256 → 128 → 64 → 32 → 16 の 5 つの全結合層を順に積みます。
3. 各隠れ層の直後に `reluLayer` を置き、最後の 16 ユニット層以外も含めて前 4 層に `dropoutLayer(0.2)` を追加します。
4. 最後に `fullyConnectedLayer(4)` と `regressionLayer` を接続し、`layerGraph` を返します。

## 6. 関数・ローカル関数詳細

### 6.1 `createDeepTireModel`

- 入力特徴量の並びはコメントにある通り `SR, SA, Fz, IA, TSTC` を想定します。
- 出力 4 成分は `Fx, Fy, Mx, Mz` を想定します。
- 正規化はネットワーク層ではなく外部前処理に任せる設計です。

## 7. 注意点

- 層構成は固定で、引数による調整はできません。
- Batch Normalization は使っていません。
