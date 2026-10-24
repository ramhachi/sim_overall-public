# `matlab/vehicle_14dof/nn_tire_predict.m`

## 概要

`nn_tire_predict` は、学習済みニューラルネットワークを用いてタイヤ力・モーメントを推定する推論インターフェースである。  
入力 `[SR, SA, Fz, IA, TSTC]` を Z-score 正規化して `predict()` へ渡し、出力を逆正規化して `Fx, Fy, Mx, Mz` を返す。

## 主要関数・クラス

- `nn_tire_predict(SR, SA, Fz, IA, TSTC, net, norm_params)`  
  NN タイヤ予測の公開関数。
- `norm_params`
  - `input_mean`, `input_std`
  - `output_mean`, `output_std`
- `predict(net, X_norm)`  
  学習済みネットワークの推論実行。

## 理論・背景

従来の TMeasy では表しにくい複雑な非線形特性を、データ駆動で近似するのがこのファイルの目的である。  
スリップ率、スリップ角、垂直荷重、キャンバー、表面温度という 5 入力から、前後力・横力・転覆モーメント・セルフアライニングトルクを同時推定する。

正規化を明示的に関数内部へ入れることで、学習時と推論時のスケール不一致を防ぎ、C++ への移植時にも同じ前処理手順を保持しやすい。  
そのため ML 系タイヤモデルの実運用インターフェースとして重要である。

## 外部ライブラリ・ツールボックス

- Deep Learning Toolbox
  - `predict`
- 標準 MATLAB 配列演算

## 入出力

- 入力
  - `SR`: スリップ率
  - `SA`: スリップ角 [deg]
  - `Fz`: 垂直荷重 [N]
  - `IA`: キャンバー角 [deg]
  - `TSTC`: タイヤ表面温度 [°C]
  - `net`: 学習済みネットワーク
  - `norm_params`: 正規化パラメータ
- 出力
  - `Fx`, `Fy`, `Mx`, `Mz`

## システム上の役割

`vehicle_14dof_nn` のタイヤサブシステムとして、物理モデルの代わりに ML 推論を供給する。  
データ駆動タイヤモデルとフルビークルダイナミクスを結び付ける要の関数である。
