# `frame_optimize/matlab/runFrameOptimization.m`

## 役割

旧 API 呼び出しを canonical frame objective へ橋渡しする **互換ラッパー**です。現在の主入口ではありません。

## 実行フロー

- 旧固定幾何引数が空であることを検証する
- canonical 向けの default config と suspension artifact を正規化する
- `frameObjectiveFunction(x, [], [], [], ...)` を目的関数にして `custom_mixed_ga` を回す
- 可能なら FEM サーバを事前起動し、終了時に停止する

## 関数詳細

### `[best_x, best_fval, history] = runFrameOptimization(nodes, elements_geom, bcs, loads, config, catalog, materials, reg_params)`
- 出力の `history` には compatibility wrapper であることと canonical entrypoint 名が入る
- 旧固定幾何入力を残したまま呼ぶと `LegacyInputsUnsupported`
- suspension artifact が無い場合は `resolve_suspension_geometry_source(..., 'ErrorIfMissing', true)` で失敗する

## 注意点

- EasyStart や `examples/run_frame_optimization.m` はこの関数を通りません。
- 旧 API を静かに受け流すのではなく、使えない引数は明示的に拒否する方向へ寄せています。
