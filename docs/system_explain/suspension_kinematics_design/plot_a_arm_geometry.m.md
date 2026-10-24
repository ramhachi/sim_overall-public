# `suspension_kinematics_design/plot_a_arm_geometry.m`

## 1. 役割

front-left hardpoints を 3D 可視化する簡易確認スクリプトです。固定の `suspension_base_geometry.json` ではなく、selector で解決した vehicle artifact から `FL` hardpoints を読みます。

## 2. 処理の流れ

1. `resolve_default_vehicle_json(..., 'RequiredHardpointCorners', {'FL'})`
2. `load_vehicle_artifact()`
3. `artifact.params.suspension.kc.corners.FL.hardpoints` 取得
4. A-arm, kingpin, wheel center, contact patch を 3D 描画

## 3. システム上の位置づけ

artifact-first へ移行したあとの front geometry spot-check 用 viewer です。
