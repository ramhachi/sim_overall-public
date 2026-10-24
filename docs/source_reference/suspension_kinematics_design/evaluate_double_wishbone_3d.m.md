# `suspension_kinematics_design/evaluate_double_wishbone_3d.m`

## 概要

ダブルウィッシュボーン幾何を 3D で評価する中核関数です。heave / roll / steer sweep を解き、KPI・caster・roll center・camber/toe curve・jacking・anti・compliance proxy を返します。

## 主な入力

- `geom`
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

## 主な出力

- `metrics`
  - static metrics
  - travel / steer / roll curves
  - roll-center migration
  - anti / jacking
  - compliance surrogate

## システム上の役割

front / rear inverse-analysis が共通で使う 3D K&C 評価 kernel です。
