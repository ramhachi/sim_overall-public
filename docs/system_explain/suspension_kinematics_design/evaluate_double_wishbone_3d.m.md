# `suspension_kinematics_design/evaluate_double_wishbone_3d.m`

## 役割

ダブルウィッシュボーン幾何を 3D で評価する新しい中心関数です。front / rear の両 optimizer がここを呼び、travel / roll / steer の sweep から runtime-facing K&C surrogate を作ります。

## 入力オプション

- `TrackWidth`
- `Wheelbase`
- `CGHeight`
- `CGToAxle`
- `StaticToe`
- `StaticCamber`
- `SteeringStiffness`
- `TravelSamples`
- `RollSamples`
- `SteerSamples`
- `UseTieRod`
- `ComplianceModel`

## 全体フロー

1. nominal pose を作る。
2. wheel-center-Z 指定の pose solver で heave 系カーブを作る。
3. steer solver で steer 系カーブを作る。
4. mirror geometry を作って roll-center を axle として評価。
5. polynomial / linear fit で runtime surrogate へ圧縮。
6. anti proxy と compliance proxy を作る。
7. static metrics とカーブ類を `metrics` にまとめて返す。

## 内部構成

- `solve_pose_for_wheel_center_z()`
- `solve_pose_for_steer()`
- `build_upright_state()`
- `compute_static_metrics()`
- `compute_axle_roll_center_height()`
- `estimate_compliance_proxy()`

## システム上の位置づけ

artifact-first suspension inverse-analysis の 3D K&C evaluator です。
