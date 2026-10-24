# `matlab/pou_hardpoint_design/geometry/define_suspension_geometry.m`

## 1. 目的

POU 最適化と順運動学で使う `geom` 構造体を作る関数です。以前は `suspension_base_geometry.json` を前輪の基準にしていましたが、現在は **vehicle artifact の suspension hardpoints** を読む前処理へ更新されています。

## 2. 入出力

- 入力:
  - `wheel_index`
  - `vehicle_params`
- 出力:
  - `geom`

## 3. 処理の流れ

1. `vehicle_params.params.mass` → `vehicle_params.mass` → 既定値の順で wheelbase / track を取る。
2. `wheel_index` から前後左右と wheel center を決める。
3. front wheel なら `params.suspension.kc.corners.FL/FR.hardpoints` を優先して読む。
4. tie-rod hardpoints が無ければ front の既定値を補う。
5. rear や artifact 未整備時は hardcoded fallback 幾何を使う。
6. upper/lower arm axis、kingpin axis、探索境界、初期値を組み立てる。

## 4. 重要点

- A-arm 側の一次情報は sidecar ではなく artifact 側です。
- POU は front hardpoints を consume する側であり、front suspension inverse-analysis の下流に位置します。

## 5. システム上の位置づけ

artifact-first front suspension geometry と POU solver 群を接続する最重要前処理です。
