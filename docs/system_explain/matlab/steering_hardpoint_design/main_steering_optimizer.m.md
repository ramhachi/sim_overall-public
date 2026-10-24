# `matlab/steering_hardpoint_design/main_steering_optimizer.m`

## 1. 役割

front suspension hardpoints を持つ artifact を入力に、タイロッド内外点を最適化して同じ artifact へ追記する Layer 1.5 の関数です。

## 2. 前提

- `params.suspension.kc.corners.FL.hardpoints` が存在すること
- 未指定時は `resolve_default_vehicle_json(..., 'RequiredHardpointCorners', {'FL'})`

## 3. 全体フロー

1. artifact を読み、baseline merge。
2. `FL.hardpoints` から `upper_ball_joint`, `lower_ball_joint`, `wheel_center` を抽出。
3. タイロッド内点 / 外点 6 変数で `fmincon + MultiStart`。
4. `eval_steer()` で bump steer / moment arm / stiffness penalty を評価。
5. `FL` にそのまま保存し、`FR` は Y 反転で mirror 保存。
6. `params.suspension_geometry.front_steering` と `optimization.stages[]` も更新。

## 4. 以前との違い

- `suspension_base_geometry.json` は使いません。
- 保存先は `params.suspension.kc.corners.FL/FR.hardpoints.tie_rod_*` です。

## 5. システム上の位置づけ

front A-arm inverse-analysis と POU / runtime steering model の間をつなぐ steering geometry 決定ステージです。
