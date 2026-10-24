# `suspension_kinematics_design/main_a_arm_optimizer.m`

## 1. 役割

front axle の runtime-driven inverse-analysis 本体です。front-left を最適化し、その鏡像から FR を作って、front K&C / hardpoints / metadata を同じ vehicle artifact へ保存します。

## 2. 入力

- `json_file`
  - 省略時は `resolve_default_vehicle_json(project_root, 'RequireExportReady', true)`

## 3. 全体フロー

1. artifact 読込と baseline merge。
2. `build_inverse_targets(p, 'front')` で target 作成。
3. 10 変数の hardpoint 最適化問題を `fmincon + MultiStart` で解く。
4. `evaluate_double_wishbone_3d()` で front-left / front-right を評価。
5. `params.suspension.kc.front` と `corners.FL/FR` を組む。
6. `params.suspension_geometry.front` に bounds / design vector / achieved values を入れる。
7. `save_suspension_geometry_artifact()` で同じ JSON に保存。

## 4. 以前との大きな違い

- `suspension_base_geometry.json` へは書きません。
- 目標値はコード内固定ではなく `build_inverse_targets()` を通じて runtime artifact から導出します。
- static KPI / caster だけでなく、RC migration や jacking surrogate も保存対象です。

## 5. システム上の位置づけ

artifact-first サスペンション設計の front 入口であり、steering / POU / frame の上流です。
