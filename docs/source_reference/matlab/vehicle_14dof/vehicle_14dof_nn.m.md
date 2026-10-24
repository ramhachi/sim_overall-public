# `matlab/vehicle_14dof/vehicle_14dof_nn.m`

## 概要

`vehicle_14dof_nn` は、TMeasy の代わりに NN タイヤモデルを使う 14DOF 派生版である。  
タイヤ表面温度 4 状態を追加し、各輪で `nn_tire_predict()` による推論と `tire_thermal_update()` による温度更新を行う。

## 主要関数・クラス

- `vehicle_14dof_nn(params, t_span, dt, input_func, nn_model, norm_params)`  
  主関数。36 状態を積分し、温度を含む出力を返す。
- `compute_equilibrium(params)`  
  平衡位置の計算。
- `derivatives(...)`  
  NN タイヤ力、熱モデル、Bouc-Wen、LuGre、ARB、バンプストップ、空力、剛体運動を統合する。
- 依存関数
  - `nn_tire_predict`
  - `tire_thermal_update`

## 理論・背景

このモデルではタイヤ力を物理パラメトリック式でなくデータ駆動で与えるため、複雑な非線形特性を取り込みやすい。  
一方で、NN に入力する表面温度も状態として持つことで、温度依存タイヤ応答の再現可能性を広げている。

状態数は 36 で、TMeasy 内部状態 `xi/eta` を持たない代わりに `T_surf` を持つ構成である。  
そのため、物理簡略モデルから ML モデルへの置換実験を行う統合プラットフォームとして使われる。

## 外部ライブラリ・ツールボックス

- Deep Learning Toolbox
  - NN 推論に必要
- そのほかは標準 MATLAB 機能

## 入出力

- 入力
  - 車両パラメータ
  - `nn_model`
  - `norm_params`
  - `t_span`, `dt`
  - `input_func`
- 出力
  - `t`
  - `state`: 36 状態履歴
  - `data`: `ax`, `ay`, `Fz`, `Fx`, `Fy`, `F_spring`, `F_bw`, `F_lugre`, `T_surf`

## システム上の役割

物理ベース 14DOF と並行する ML 強化版フルビークルモデルである。  
将来の学習済みタイヤモデル活用や C++ 推論統合の評価母体として、研究用途で重要な位置を占める。
