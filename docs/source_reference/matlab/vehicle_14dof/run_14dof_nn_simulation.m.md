# `matlab/vehicle_14dof/run_14dof_nn_simulation.m`

## 概要

`run_14dof_nn_simulation.m` は、学習済み NN タイヤモデルを用いた 14DOF 車両シミュレーション実行スクリプトである。  
`ultra_deep_validation_model.mat` を読み込み、`vehicle_14dof_nn` を走らせ、車両応答とタイヤ温度を含む結果を CSV 化する。

## 主要関数・クラス

- モデル読込部  
  `../dl_tire_model/output/ultra_deep_validation_model.mat` から `trained_net` と `norm_params` を取得する。
- `vehicle_14dof_nn(...)`  
  NN タイヤ版 14DOF モデル本体。
- 結果表示部  
  位置、姿勢、速度、輪荷重、ダンパ力、タイヤ表面温度などを表示する。
- CSV 出力部  
  `14dof_nn_result.csv` を保存する。

## 理論・背景

物理タイヤモデルの代わりに学習済みネットワークを使うことで、実測データ由来の複雑なタイヤ挙動をフルビークルへ反映できる。  
さらにタイヤ表面温度を状態として持つため、温度によってタイヤ応答が変わる拡張系への足掛かりにもなる。

このスクリプトは ML と物理モデルの結合試験という意味合いが強く、NN 推論・温度更新・車両 ODE が一緒に安定動作するかを確認する場である。  
そのため結果出力も車両運動だけでなく温度まで含めている。

## 外部ライブラリ・ツールボックス

- Deep Learning Toolbox
  - 学習済みネットワークの読込・推論
- 標準 MATLAB I/O / 描画機能

## 入出力

- 入力
  - `ultra_deep_validation_model.mat`
  - 5 秒解析、`dt=1 ms`
  - `t=1 s` で `0.1 rad` ステア
- 出力
  - `14dof_nn_result.csv`
  - コンソール上の最終姿勢、輪荷重、ダンパ力、タイヤ表面温度

## システム上の役割

ML 強化版フルビークルシミュレーションの実行パイプラインを担う。  
従来 TMeasy 系とは別系統で、NN タイヤモデル統合の実証・比較・将来の C++ 移植検証の起点となる。
