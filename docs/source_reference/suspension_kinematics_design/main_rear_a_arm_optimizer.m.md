# `suspension_kinematics_design/main_rear_a_arm_optimizer.m`

## 概要

rear axle 用の runtime-driven 逆解析入口です。rear-left を最適化し、mirror した `RR` を含めて rear hardpoints / K&C surrogate / metadata を同じ車両 artifact に追記します。

## 主な処理

- `build_inverse_targets(..., 'rear')`
- `fmincon + MultiStart`
- `evaluate_double_wishbone_3d()` による rear 評価
- `save_suspension_geometry_artifact()` による `params.suspension.kc.rear` / `corners.RL/RR` 更新

## システム上の役割

frame 側が参照する rear hardpoints の供給元であり、front と対になる rear inverse-analysis ステージです。
