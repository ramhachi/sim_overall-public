# `suspension_kinematics_design/main_a_arm_optimizer.m`

## 概要

front axle 用の runtime-driven 逆解析入口です。車両 artifact から目標値を読み、フロント左右の hardpoints・K&C surrogate・inverse-analysis metadata を**同じ artifact**へ保存します。

## 主な処理

- `resolve_default_vehicle_json()` で既定 artifact を解決
- `build_inverse_targets(..., 'front')` で目標を生成
- `fmincon + MultiStart` で front-left 幾何を探索
- `evaluate_double_wishbone_3d()` で 3D 指標を評価
- `save_suspension_geometry_artifact()` で `params.suspension.kc.front` / `corners.FL/FR` / `params.suspension_geometry.front` を更新

## システム上の役割

artifact-first サスペンション設計の front 入口です。後段の steering、POU、frame はこの出力 hardpoints を参照します。
