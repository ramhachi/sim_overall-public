# evaluateModel.m

## 概要
学習済みネットワークを検証データで評価し、RMSE、R²、TIC、MAE などをまとめた指標構造体を返す関数です。正規化解除も含めて、物理単位での性能確認を行います。

## 主要関数・クラス
- `evaluateModel(trained_net, X_val, Y_val, norm_params)`
- 内部で `predict`、`denormalizeData`、`computeRMSE`、`computeR2`、`computeTIC` などを使用

## 理論・背景
学習損失だけでは物理量ごとの精度を十分に判断できません。本関数は複数指標を併用し、絶対誤差、分散説明率、スケール不変誤差を同時に確認できるように設計されています。

## 外部ライブラリ・ツールボックス
- Deep Learning Toolbox
- MATLAB 標準統計・数値演算

## 入出力
- 入力:
  - `trained_net`: 学習済みネットワーク
  - `X_val`, `Y_val`: 検証データ
  - `norm_params`: 入出力の平均・標準偏差
- 出力: Fx/Fy/Mx/Mz ごとの詳細指標と要件判定を含む構造体

## システム上の役割
学習結果の正式な品質判定関数です。メインワークフロー、ハイパーパラメータ探索、最終検証、比較モデル評価の基準点になります。
