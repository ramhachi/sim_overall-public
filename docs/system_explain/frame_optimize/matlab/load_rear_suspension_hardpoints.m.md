# `frame_optimize/matlab/load_rear_suspension_hardpoints.m`

## 役割

rear suspension hardpoints を持つ vehicle artifact から、rear chassis mount と可視化用 upright 点群を frame 座標へ変換する関数です。

## 処理の流れ

1. `load_vehicle_artifact(json_path)`。
2. `params.suspension.kc.corners.RL.hardpoints` を検証。
3. `[-x + wheelbase_m, y, -z]` で前軸基準の frame 座標へ変換。
4. `abs(y)` で RR 側を鏡映生成。
5. `rear_mount_nodes` と `rear_upright_nodes` を順序付きで返す。

## 重要点

- 入力は `suspension_base_geometry.json` ではなく artifact です。
- `RL` hardpoints が無い場合は warning で失敗扱いになります。

## システム上の位置づけ

frame 最適化が rear mount を得るための artifact reader です。
