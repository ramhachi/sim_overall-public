# trainTireModel.m

## 概要
本ディレクトリの学習エンジン本体です。ネットワーク、学習データ、検証データ、設定構造体を受け取り、GPU 利用判定、`trainingOptions` 作成、学習実行、発散検出、GPU メモリ不足時のリトライ、CPU フォールバックまでを一括で処理します。

## 主要関数・クラス
- `trainTireModel(net, X_train, Y_train, X_val, Y_val, config)`
- `checkGPUAvailability`
- `createTrainingOptions`
- `trainNetwork`

## 理論・背景
深層学習では、理想的な設定でも実行環境差やデータ状況によって発散やメモリ不足が起こります。本関数は単なる `trainNetwork` のラッパーではなく、実験を最後まで進めるための耐障害性を備えています。具体的には NaN 損失による発散検出、GPU out-of-memory 時のバッチサイズ縮小、必要なら CPU への切替を行い、研究環境でも実用環境でも同一コードを回しやすくしています。

## 外部ライブラリ・ツールボックス
- Deep Learning Toolbox
- Parallel Computing Toolbox（GPU 利用時）

## 入出力
- 入力:
  - `net`: 生成済みネットワーク
  - `X_train`, `Y_train`: 学習データ
  - `X_val`, `Y_val`: 検証データ
  - `config`: エポック数、学習率、バッチサイズ、patience など
- 出力:
  - `trained_net`
  - `training_info`（損失履歴、最終検証損失、学習時間など）

## システム上の役割
すべての学習系ワークフローの中心です。モデル生成と評価の間に位置し、実験の安定実行と再現性確保を担います。
