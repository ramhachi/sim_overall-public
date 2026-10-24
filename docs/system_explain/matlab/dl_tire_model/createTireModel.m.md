# `matlab/dl_tire_model/createTireModel.m`

## 1. 概要

`createTireModel` は、5 入力・4 出力の標準的な全結合回帰ネットワークを `layerGraph` として返す関数です。

## 2. ファイルの役割

- タイヤモデル学習で使う基本の feedforward アーキテクチャを定義します。
- `createModelFromConfig` の `feedforward` 分岐先として利用されます。

## 3. インターフェース

### 3.1 関数シグネチャ

```matlab
net = createTireModel()
```

### 3.2 戻り値

- `net`
  - `layerGraph(layers)` の結果

## 4. 依存関係

- Deep Learning Toolbox 関数
  - `featureInputLayer`
  - `fullyConnectedLayer`
  - `reluLayer`
  - `dropoutLayer`
  - `regressionLayer`
  - `layerGraph`

## 5. 処理の流れ

1. `featureInputLayer(5, 'Normalization', 'none')` を定義します。
2. `128 -> 64 -> 32` の 3 つの全結合隠れ層を積みます。
3. 第 1・第 2 隠れ層の後ろに `reluLayer` と `dropoutLayer(0.2)` を置きます。
4. 第 3 隠れ層の後ろに `reluLayer` を置きます。
5. 最後に `fullyConnectedLayer(4)` と `regressionLayer` を付けて `layerGraph` を返します。

## 6. 関数・ローカル関数詳細

### 6.1 `createTireModel`

- 入力の列順は `SR, SA, Fz, IA, TSTC` を想定しています。
- 出力 4 成分は `Fx, Fy, Mx, Mz` を想定しています。
- 入力層の `Normalization` は `'none'` なので、正規化は外部前処理が前提です。

## 7. 注意点

- 層サイズやドロップアウト率を引数で変える仕組みはありません。
- Batch Normalization や残差接続は使っていません。
