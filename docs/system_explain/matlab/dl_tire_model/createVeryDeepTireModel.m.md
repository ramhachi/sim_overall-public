# `matlab/dl_tire_model/createVeryDeepTireModel.m`

## 1. 概要

`createVeryDeepTireModel` は、8 隠れ層を持つ非常に深い全結合回帰ネットワークを生成し、作成内容をコンソールへ表示する関数です。

## 2. ファイルの役割

- 実験的な高容量モデルを定義します。
- 各全結合ブロックに Batch Normalization と ReLU を入れます。
- `256-256-128-128-64-64-32-32` の固定構成を返します。

## 3. インターフェース

### 3.1 関数シグネチャ

```matlab
net = createVeryDeepTireModel()
```

### 3.2 戻り値

- `net`
  - 非常に深い feedforward モデルの `layerGraph`

## 4. 依存関係

- Deep Learning Toolbox 関数
  - `featureInputLayer`
  - `fullyConnectedLayer`
  - `batchNormalizationLayer`
  - `reluLayer`
  - `dropoutLayer`
  - `regressionLayer`
  - `layerGraph`
- MATLAB 標準関数: `fprintf`

## 5. 処理の流れ

1. 作成開始メッセージを表示します。
2. 入力層 `featureInputLayer(5, 'Normalization', 'none')` を定義します。
3. 8 つの全結合ブロックを順に並べます。
   - 256, 256, 128, 128, 64, 64, 32, 32
4. 各ブロックで `fullyConnectedLayer` → `batchNormalizationLayer` → `reluLayer` の順に接続します。
5. ドロップアウトはブロック 1〜4 で `0.2`、ブロック 5〜6 で `0.15`、ブロック 7 で `0.1`、ブロック 8 では未使用です。
6. 最後に `fullyConnectedLayer(4)` と `regressionLayer` を置き、`layerGraph` を返します。
7. 作成後に層数・ドロップアウト・概算パラメータ数などを表示します。

## 6. 関数・ローカル関数詳細

### 6.1 `createVeryDeepTireModel`

- 出力メッセージでは `Total parameters: ~200k` と表示しますが、これは計算ではなく固定文字列です。
- 正規化は層で行わず、外部前処理前提です。

## 7. 注意点

- 説明コメントは実験的アプローチとして位置付けていますが、性能評価はこのファイル内では行いません。
- 層構成は固定で、設定構造体などによる変更はできません。
