# `frame_optimize/matlab/resolve_suspension_geometry_source.m`

## 役割

frame 側が読むべき authoritative suspension artifact を決める resolver です。以前の hardcoded sidecar 依存をやめ、shared candidate selector を通して `FL` / `RL` hardpoints を持つ artifact だけを受け入れます。

## 処理の流れ

1. `find_vehicle_json_candidates` が見えなければ root `matlab/` を path 追加。
2. `preferred_file` が有効かつ hardpoints を持てばそれを返す。
3. そうでなければ `find_vehicle_json_candidates(..., 'RequiredHardpointCorners', {'FL','RL'})` を呼ぶ。
4. 候補があれば最上位を返す。
5. 無ければ、`ErrorIfMissing=true` なら `MissingGeometry` で止め、そうでなければ warning を出して空文字を返す。

## 補助関数

### `has_artifact_hardpoints(json_path)`
- artifact を読んで `kc.corners.FL` / `RL` の hardpoints 有無を調べる。

## システム上の位置づけ

 frame 最適化を suspension artifact SSOT に接続する最初の分岐点です。
