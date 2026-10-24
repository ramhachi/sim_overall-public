# `matlab/dl_tire_model/createModelFromConfig.m`

## 1. 概要

`createModelFromConfig` は、設定構造体の `model_type` を見て、全結合モデルか LSTM モデルかを選択して返す関数です。

## 2. ファイルの役割

- `config.model_type` に応じてネットワーク生成関数を振り分けます。
- モデル生成の分岐点を 1 箇所にまとめます。

## 3. インターフェース

### 3.1 関数シグネチャ

```matlab
net = createModelFromConfig(config)
```

### 3.2 引数

- `config`
  - 少なくとも `model_type` を持つ構造体を想定
  - `model_type` 未定義時は `'feedforward'` が補われます。

### 3.3 戻り値

- `net`
  - `createTireModel()` または `createLSTMTireModel()` の返り値

## 4. 依存関係

- プロジェクト関数
  - `createTireModel`
  - `createLSTMTireModel`
- MATLAB 標準関数: `isfield`, `lower`, `error`

## 5. 処理の流れ

1. `config.model_type` が存在しない場合は `'feedforward'` を設定します。
2. `switch lower(config.model_type)` で分岐します。
3. `'feedforward'` の場合は `createTireModel()` を返します。
4. `'lstm'` の場合は `createLSTMTireModel()` を返します。
5. それ以外の値なら `error` を投げます。

## 6. 関数・ローカル関数詳細

### 6.1 `createModelFromConfig`

- 分岐に使う設定項目は `model_type` だけです。
- `config` に他のフィールドがあっても、この関数では参照しません。

## 7. 注意点

- `hidden_sizes` や `dropout_rate` を含む設定構造体を渡しても、この関数単体では反映しません。
- `lower` を通すため、`'LSTM'` のような大文字混在も受け付けます。
