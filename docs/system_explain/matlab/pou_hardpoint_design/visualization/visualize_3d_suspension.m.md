# `matlab/pou_hardpoint_design/visualization/visualize_3d_suspension.m`

## 1. 目的

vehicle artifact に保存された A-arm / tie-rod hardpoints と、`all_results` の POU 最適化結果をまとめて 3D 表示する関数です。

## 2. 入出力

- 入力:
  - `all_results`
  - `vehicle_json_path`（省略可）
- 出力:
  - 戻り値なし。figure を生成。

## 3. 全体処理フロー

1. `vehicle_json_path` が無ければ `resolve_default_vehicle_json(..., 'RequireSuspensionHardpoints', true)`。
2. artifact を読み、wheelbase を取得。
3. `params.suspension.kc.corners.*.hardpoints` から各 corner の描画用 geometry を組む。
4. `all_results` 側に POU 結果があればその wheel に pushrod / rocker / damper を追加描画。
5. A-arm、kingpin、tie-rod、POU、タイヤを 1 枚へ重ねる。

## 4. 以前との違い

- `suspension_base_geometry.json` を再読込しません。
- A-arm hardpoints の一次情報は artifact 側です。

## 5. システム上の位置づけ

artifact-first サスペンション設計結果を空間形状としてレビューする最終可視化層です。
