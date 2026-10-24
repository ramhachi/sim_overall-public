# `suspension_kinematics_design/main_rear_a_arm_optimizer.m`

## 1. 役割

rear axle の runtime-driven inverse-analysis 本体です。rear-left を最適化し、mirror で RR を作って rear hardpoints / K&C / metadata を artifact へ追記します。

## 2. 入力

- `json_file`
  - 省略時は `resolve_default_vehicle_json(project_root, 'RequireExportReady', true)`

## 3. 全体フロー

1. artifact 読込と baseline merge。
2. `build_inverse_targets(p, 'rear')` で target 作成。
3. rear 用 bounds / 初期値で `fmincon + MultiStart`。
4. `evaluate_double_wishbone_3d()` で RL / RR を評価。
5. `params.suspension.kc.rear` と `corners.RL/RR` を更新。
6. `params.suspension_geometry.rear` に solver metadata を保存。
7. `save_suspension_geometry_artifact()` で同じ JSON に書く。

## 4. front と違う点

- `CGToAxle` に rear 側距離 `b` を使う。
- anti 指標は rear 側 (`anti_drive_pct`, `anti_rise_pct`) を主に評価に入れる。

## 5. システム上の位置づけ

frame 最適化が読む RL / RR hardpoints の供給元であり、front inverse-analysis と対になる rear ステージです。
