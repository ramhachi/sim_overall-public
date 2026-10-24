# createDeepTireModel.m

## 概要
5 層の隠れ層を持つ比較的深い全結合ニューラルネットワークを生成する関数です。構成は `256-128-64-32-16` で、標準モデルより高い表現力を狙います。

## 主要関数・クラス
- `createDeepTireModel()`
- `featureInputLayer`
- `fullyConnectedLayer`
- `reluLayer`
- `dropoutLayer`
- `regressionLayer`

## 理論・背景
タイヤの複合スリップ応答は非線形性が強く、単純な浅いネットワークではピーク近傍や荷重依存性を十分に表現しづらい場合があります。本モデルは層深さを増やし、非線形写像の自由度を拡張する設計です。

## 外部ライブラリ・ツールボックス
- Deep Learning Toolbox

## 入出力
- 入力: なし
- 出力: `layerGraph` 形式のネットワーク

## システム上の役割
標準 `createTireModel` より高性能を狙う代替アーキテクチャです。深さの効果を比較する検証や改良学習ワークフローで使われます。
