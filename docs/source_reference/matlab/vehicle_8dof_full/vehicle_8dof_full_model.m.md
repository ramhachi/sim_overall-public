# `matlab/vehicle_8dof_full/vehicle_8dof_full_model.m`

## 概要

`vehicle_8dof_full_model` は、8DOF 車両モデルに Bouc-Wen ダンパ、LuGre 摩擦、ARB を統合した 40 状態のサスペンション高詳細モデルである。  
各輪について Bouc-Wen 履歴変数と LuGre ブリッスル変形を状態として持ち、ダンパ力と摩擦力を別々に計算・記録する。

## 主要関数・クラス

- `vehicle_8dof_full_model(params, t_span, dt, input_func)`  
  主関数。40 状態を RK4 で積分し、`F_damper_bw` と `F_friction_lugre` を返す。
- `compute_equilibrium(params)`  
  静的平衡位置の算出。
- `derivatives(...)`  
  Bouc-Wen、LuGre、タイヤ、ばね、空力、駆動、ばね下運動を一括評価する。
- `compute_arb_forces(...)`  
  左右ストローク差から ARB 力を求める。

## 理論・背景

LuGre モデルは、摩擦を単なるクーロン項ではなく、接触表面の微視的毛変形として表す。  
本ファイルでは  
`g(v) = Fc + (Fs - Fc) * exp(-(v/vs)^2)`  
を Stribeck 関数として使い、  
`dz/dt = v - (sigma0*|v|/g(v))*z`  
によりブリッスル変形を更新する。

摩擦力は `sigma0*z + sigma1*dz/dt + sigma2*v` で求められ、Bouc-Wen ダンパ力と合成してサスペンション総減衰を構成する。  
このため、ヒステリシスとプレ・スライディング摩擦を同時に表現でき、実車に近い微小速度域応答を扱いやすい。

## 外部ライブラリ・ツールボックス

外部ツールボックスは不要。  
非線形摩擦・ヒステリシス・積分処理はすべて標準 MATLAB 機能上で自前実装される。

## 入出力

- 入力
  - `params`: Bouc-Wen、LuGre、ARB、タイヤ、空力、駆動、質量パラメータ
  - `t_span`, `dt`
  - `input_func`
- 出力
  - `t`
  - `state`: 40 状態履歴
  - `data`: `ax`, `ay`, `Fz`, `F_damper_bw`, `F_friction_lugre`

## システム上の役割

完全車両統合前にサスペンション詳細モデルを単独で成熟させるための中心ファイルである。  
14DOF モデルへ移行する前に、Bouc-Wen と LuGre の寄与を明示的に追跡できる検証基盤を提供する。
