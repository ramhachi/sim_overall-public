# `matlab/vehicle_6dof/vehicle_6dof_model.m`

## 概要

`vehicle_6dof_model` は、ばね上剛体の 6DOF 運動に 4 輪タイヤ力と車輪回転を組み合わせた車両モデルである。  
状態は 24 個で、位置・オイラー角・並進速度・角速度・車輪回転・タイヤ内部状態を RK4 で積分する。

## 主要関数・クラス

- `vehicle_6dof_model(params, t_span, dt, input_func)`  
  主関数。静的平衡を求めて初期状態を構成し、時系列を積分する。
- `compute_equilibrium(params)`  
  静的輪荷重 `static_Fz` と平衡沈み込み `z_eq` を計算する。
- `derivatives(...)`  
  剛体 6DOF 運動、タイヤ力、サスペンションばね・ダンパ、空力、駆動トルクをまとめて扱う。
- `euler_to_rotation_matrix(phi, theta, psi)`  
  Z-Y-X オイラー角から回転行列を作る。
- `angular_velocity_to_euler_rate(phi, theta)`  
  角速度 `p,q,r` をオイラー角速度へ写像する。
- `tmeasy_force(...)`  
  タイヤ力と内部状態微分を計算する。

## 理論・背景

このモデルは SAE J670e 座標系を前提に、車体座標系内の重力成分まで含めて剛体運動を解いている。  
姿勢表現には Tait-Bryan の Z-Y-X（yaw-pitch-roll）を用い、慣性系位置更新と車体座標系速度更新を分離している。

サスペンションはまだ独立ばね下を持たず、各輪位置でのばね上変位から輪荷重を近似計算する段階である。  
そのため 8DOF / 14DOF ほど詳細ではないが、ロール・ピッチ・ヒーブを含む基盤として有効で、上位モデルの原型になる。

## 外部ライブラリ・ツールボックス

外部ツールボックスは不要。  
回転変換、重力投影、RK4、TMeasy 力計算はすべて MATLAB 標準機能で記述されている。

## 入出力

- 入力
  - `params`: 質量・慣性・サスペンション・空力・タイヤ・パワートレイン
  - `t_span`, `dt`
  - `input_func(t)`: `steering`, `throttle`
- 出力
  - `t`
  - `state`: 24 状態の履歴
  - `data`: `ax`, `ay`, `Fz`

## システム上の役割

3DOF から 8/14DOF へ進む橋渡しとして、立体姿勢運動を含む車両ダイナミクス基盤を提供する。  
空力、駆動、姿勢変換、タイヤモデルの統合点として、上位自由度モデルの設計確認に重要な位置を占める。
