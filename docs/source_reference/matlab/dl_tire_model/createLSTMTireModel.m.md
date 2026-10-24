# createLSTMTireModel.m

## 概要
時系列依存を扱うための LSTM ベースタイヤモデルを生成する関数です。系列入力から最終的に Fx、Fy、Mx、Mz を回帰します。

## 主要関数・クラス
- `createLSTMTireModel()`
- `sequenceInputLayer`
- `lstmLayer`
- `dropoutLayer`
- `fullyConnectedLayer`
- `regressionLayer`

## 理論・背景
タイヤ応答には履歴依存や過渡性が存在しうるため、独立サンプル前提の全結合ネットでは捉えにくい場合があります。LSTM 版はその可能性を検証するための代替アーキテクチャです。

## 外部ライブラリ・ツールボックス
- Deep Learning Toolbox

## 入出力
- 入力: なし
- 出力: LSTM ベースの `layerGraph`

## システム上の役割
`createModelFromConfig` から選択される分岐の一つです。標準の静的回帰モデルと比較し、系列情報が有効かどうかを検証するために用いられます。
