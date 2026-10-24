# `matlab/pou_hardpoint_design/visualization/visualize_3d_suspension.m`

## 概要

vehicle artifact に保存された A-arm / tie-rod hardpoints と、`all_results` に入る POU 結果を重ねて 3D 表示する可視化関数です。

## 主な入力

- `all_results`
- `vehicle_json_path`（省略時は `resolve_default_vehicle_json(..., 'RequireSuspensionHardpoints', true)`）

## 重要な挙動

- `params.suspension.kc.corners.*.hardpoints` を直接読みます。
- front は tie-rod を、rear は A-arm / upright を描きます。
- `M_pivot_chassis` など POU 結果がある wheel だけ rocker / damper を追加描画します。

## システム上の役割

artifact-first サスペンション設計結果を CAD 前に俯瞰確認する最終可視化層です。
