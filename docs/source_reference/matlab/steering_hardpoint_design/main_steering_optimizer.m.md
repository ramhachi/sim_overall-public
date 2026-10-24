# `matlab/steering_hardpoint_design/main_steering_optimizer.m`

## 概要

front hardpoints を持つ車両 artifact を入力に、タイロッド内外点を最適化して同じ artifact へ追記する Layer 1.5 の最適化入口です。

## 主な入力

- `json_file`
  - 未指定時は `resolve_default_vehicle_json(..., 'RequiredHardpointCorners', {'FL'})`

## 主な出力

- `params.suspension.kc.corners.FL/FR.hardpoints.tie_rod_inner`
- `params.suspension.kc.corners.FL/FR.hardpoints.tie_rod_outer`
- `params.suspension_geometry.front_steering`
- `optimization.stages[]`

## 重要な挙動

- 前提として `FL` hardpoints が artifact に存在している必要があります。
- `fmincon + MultiStart` で bump steer / moment arm / stiffness penalty を最小化します。
- FR 側は FL 解を左右反転して保存します。

## システム上の役割

front A-arm inverse-analysis と POU / runtime steering の間をつなぐ steering hardpoint 決定ステージです。
