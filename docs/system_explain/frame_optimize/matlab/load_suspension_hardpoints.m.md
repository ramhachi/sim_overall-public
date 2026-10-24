# `frame_optimize/matlab/load_suspension_hardpoints.m`

## 役割

front suspension hardpoints を持つ vehicle artifact から、frame 最適化で使う前側取付点と FBH 4 点を生成する関数です。

## 処理の流れ

1. FBH 寸法の既定値を設定。
2. `load_vehicle_artifact(json_path)` で artifact を読む。
3. `params.suspension.kc.corners.FL.hardpoints` を取り出す。
4. `[-x, y, -z]` で frame 座標へ変換。
5. `abs(y)` により FR 側を鏡映生成。
6. `wheel_center` を基準に FBH 位置も作る。

## 重要点

- sidecar JSON ではなく artifact hardpoints 前提です。
- `FL` hardpoints が無ければ warning で失敗扱いになります。

## システム上の位置づけ

frame 側 front mount 生成の入口です。`resolve_suspension_geometry_source()` と対で使われます。
