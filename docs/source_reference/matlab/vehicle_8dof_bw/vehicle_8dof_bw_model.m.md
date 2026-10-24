# `matlab/vehicle_8dof_bw/vehicle_8dof_bw_model.m`

## 概要

`vehicle_8dof_bw_model` は、8DOF 車両モデルに Bouc-Wen ダンパと前後アンチロールバーを追加した 36 状態モデルである。  
各輪のダンパ履歴変数 `damper_z` を状態として持ち、ばね・Bouc-Wen・ARB が結合したサスペンション力を通して車体姿勢と輪荷重を計算する。

## 主要関数・クラス

- `vehicle_8dof_bw_model(params, t_span, dt, input_func)`  
  主関数。36 状態を RK4 で積分し、`F_damper_bw` を含む記録データを返す。
- `compute_equilibrium(params)`  
  静的輪荷重と平衡沈み込みを算出する。
- `derivatives(...)`  
  ばね、Bouc-Wen ダンパ、タイヤ、空力、駆動、ばね下運動を計算する。
- `compute_arb_forces(susp_travel, params)`  
  前後 ARB トルクを左右輪力へ換算する。

## 理論・背景

Bouc-Wen モデルはダンパのヒステリシスを少数パラメータで表現できる代表的非線形モデルである。  
本実装では  
`F = c0*v + k0*x + alpha*z`、  
`dz/dt = A*v - beta*|v|*z*|z|^(n-1) - gamma*v*|z|^n`  
の形で履歴状態を進める。

さらに ARB は左右輪のサスペンションストローク差からロール角相当量を作り、トーションばねとして左右逆向きの力を与える。  
そのため、ロール剛性の増加とヒステリシス減衰の双方を同時に観察でき、実車に近いサスペンション応答の第一段階となる。

## 外部ライブラリ・ツールボックス

外部ツールボックスは不要。  
数値積分、Bouc-Wen 更新、ARB 力計算は MATLAB 標準機能のみで実装されている。

## 入出力

- 入力
  - `params`: Bouc-Wen 係数、ARB 剛性、タイヤ・質量・空力・駆動パラメータ
  - `t_span`, `dt`
  - `input_func`
- 出力
  - `t`
  - `state`: 36 状態履歴
  - `data`: `ax`, `ay`, `Fz`, `F_damper_bw`

## システム上の役割

サスペンションの非線形ダンパ挙動を全車両運動へ結合する中核モデルである。  
後続の LuGre 摩擦統合や 14DOF 完全モデルに進む前に、Bouc-Wen 単独効果を分離して評価する役割を持つ。
