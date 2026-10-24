# frame_optimize/matlab/runFrameOptimization.m

## 役割

旧 API 呼び出しを **canonical frame objective** へ橋渡しする互換ラッパーです。新規の主入口ではなく、`nodes` / `elements_geom` / `bcs` / `loads` を直接受ける旧契約を明示的に縮退させています。

## 主な処理

- 旧固定幾何引数が空であることを `validateLegacyContract` で確認する
- geometry / suspension artifact / default penalty 設定を canonical 向けに正規化する
- `frameObjectiveFunction(x, [], [], [], ...)` を目的関数にして `custom_mixed_ga` を呼ぶ
- 可能なら FEM サーバを事前起動し、終了時に停止する
- 結果を `history` にまとめ、canonical entrypoint 名も記録する

## 入出力

- 入力: 旧互換の引数列、設定、カタログ、材料、規則
- 出力: `best_x`、`best_fval`、`history`

## 現在の位置づけ

- **EasyStart や `examples/run_frame_optimization.m` の現行経路では使いません。**
- suspension artifact が無い場合は `resolve_suspension_geometry_source(..., 'ErrorIfMissing', true)` で止まります。
- 旧固定幾何入力を残したまま呼ぶと `LegacyInputsUnsupported` でエラーになります。
