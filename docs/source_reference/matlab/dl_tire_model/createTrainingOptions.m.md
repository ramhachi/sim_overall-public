# createTrainingOptions.m

## 概要
学習設定構造体から `trainingOptions` を構築する関数です。Adam 最適化、学習率スケジュール、早期終了、検証頻度、実行環境を一括設定します。

## 主要関数・クラス
- `createTrainingOptions(config, X_val, Y_val)`
- `trainingOptions('adam', ...)`

## 理論・背景
学習の安定性は最適化器とスケジュール設計に強く依存します。本関数では初期学習率、一定周期での減衰、検証損失に基づく早期終了を定義し、過学習と発散の両方を抑えます。

## 外部ライブラリ・ツールボックス
- Deep Learning Toolbox

## 入出力
- 入力:
  - `config`: 学習率、バッチサイズ、エポック数、patience など
  - `X_val`, `Y_val`: 検証データ
- 出力: `trainNetwork` に渡すオプションオブジェクト

## システム上の役割
`trainTireModel` の設定集約点です。ワークフロー側では個別オプションを意識せず、構造体だけで学習条件を切り替えられます。
