# createModelFromConfig.m

## 概要
設定構造体に応じて適切なネットワーク生成関数を呼び出すファクトリ関数です。少なくとも feedforward と LSTM の選択に対応します。

## 主要関数・クラス
- `createModelFromConfig(config)`
- 呼び出し先: `createTireModel`, `createLSTMTireModel` など

## 理論・背景
ワークフロー側でモデル生成ロジックを分岐させると、学習コードが煩雑になります。本関数はアーキテクチャ選択を一点に集約し、探索処理やメインワークフローを単純化します。

## 外部ライブラリ・ツールボックス
- Deep Learning Toolbox

## 入出力
- 入力: `config.model_type`
- 出力: 選択された `layerGraph`

## システム上の役割
モデル種別の抽象化レイヤです。ハイパーパラメータ探索、通常学習、比較実験のすべてが同じインターフェースでネットワーク生成を行えるようにします。
