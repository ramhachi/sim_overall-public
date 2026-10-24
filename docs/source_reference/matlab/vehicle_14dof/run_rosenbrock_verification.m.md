# `matlab/vehicle_14dof/run_rosenbrock_verification.m`

## 概要

`run_rosenbrock_verification.m` は、固定刻み RK4 と MATLAB の `ode23s`（Rosenbrock 系解法）を比較する積分器評価スクリプトである。  
14DOF 完全車両モデルを対象に、`dt=1 ms` と `0.1 ms` の RK4、ならびに適応刻み `ode23s` を実行し、精度・ステップ数・計算時間を比較する。

## 主要関数・クラス

- `vehicle_14dof_model(...)`  
  RK4 の比較対象。
- `ode23s(...)`  
  剛性 ODE 向けの MATLAB ODE Suite ソルバ。
- `odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',1e-3)`  
  `ode23s` の許容差設定。
- 比較部  
  最終位置、最終速度、最大ロール角、計算時間、ステップ数、基準解との差を集計する。

## 理論・背景

Rosenbrock 系解法は剛性を持つ ODE に強い一方、実装と運用が複雑になる。  
本ファイルは、現在の 14DOF 系で本当に適応剛性解法が必要か、それとも 1 ms RK4 で十分かを判断するための数値実験を行う。

高精度基準として `dt=0.1 ms` RK4 を置き、それに対する 1 ms RK4 と `ode23s` の誤差を見る構成のため、実務的な判断材料になりやすい。  
計算時間も並べることで、精度と速度のトレードオフを明確化している。

## 外部ライブラリ・ツールボックス

- MATLAB ODE Suite
  - `ode23s`
  - `odeset`
- そのほかは標準描画機能

## 入出力

- 入力
  - 10 秒解析
  - RK4: `dt=1 ms`, `0.1 ms`
  - `ode23s`: `RelTol=1e-6`, `AbsTol=1e-8`
- 出力
  - 比較図 `../../output/rosenbrock_comparison.png`
  - 計算時間、ステップ数、最終状態、誤差評価

## システム上の役割

積分器選定の技術判断資料を生成するファイルである。  
現行 14DOF モデルにおける RK4 採用妥当性や、将来的に剛性ソルバを導入すべきかの根拠を与える。
