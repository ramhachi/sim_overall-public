# createVeryDeepTireModel.m

## 概要
8 隠れ層・約 20 万パラメータ規模の非常に深い全結合ネットワークを構築する実験用関数です。構成は `256-256-128-128-64-64-32-32` で、各段にバッチ正規化と段階的ドロップアウトを組み込みます。

## 主要関数・クラス
- `createVeryDeepTireModel()`
- `featureInputLayer`
- `fullyConnectedLayer`
- `batchNormalizationLayer`
- `reluLayer`
- `dropoutLayer`
- `regressionLayer`

## 理論・背景
複合スリップ、荷重依存、温度依存、キャンバ依存を同時に近似するには高い表現力が必要です。一方で深いネットワークは勾配不安定や過学習の危険もあるため、バッチ正規化とドロップアウトで訓練安定性を補っています。深さを増やすことでどこまで誤差を削減できるかを検証する、研究寄りのアーキテクチャです。

## 外部ライブラリ・ツールボックス
- Deep Learning Toolbox

## 入出力
- 入力: なし
- 出力: 8 隠れ層の `layerGraph`

## システム上の役割
高精度志向の検証系スクリプトで使われる主力候補です。`run_ultra_deep_validation` などで深さの効果を評価し、C++ へ持ち込む最終モデル候補の一つになります。
