# `frame_optimize/matlab/run_canonical_frame_optimization.m`

## 役割

現行の **canonical frame optimization API** です。artifact 解決、最適化設定、FEM サーバ起動、GA 実行、可視化、artifact 保存、任意の CAD 出力までを **非対話で** 実行します。

## 主な処理

1. `resolve_authoritative_vehicle_json` と `resolve_suspension_geometry_source(..., 'ErrorIfMissing', true)` で入力 artifact を確定する
2. geometry / mass / `K_chassis` を vehicle artifact から反映する
3. exclusion zone と cockpit zone、penalty 設定、fail-fast フラグを `config` に組む
4. `custom_mixed_ga` で `frameObjectiveFunction` を探索する
5. 最良解を再生成し、`fsae_viewer` と任意の SolidWorks / Onshape 向け CAD 出力を行う
6. `save_frame_optimization_artifact` で `params.frame` と `K_chassis` を unified vehicle artifact へ保存する

## 主なオプション

- `VehicleParamsFile`: 更新対象または基準にする artifact
- `MassTargetKg`: 目標重量ペナルティの有効化
- `GenerateSolidWorksMacro`: VBA マクロ出力の有無
- `GenerateOnshape3DSketch`: Onshape 用 FeatureScript 出力の有無
- `WaitForFigureClose`: 図ウィンドウを閉じるまで待つか
- `PopulationSize`, `MaxGenerations`, `UseParallel`: GA 実行設定

## 出力

- `best_x`, `best_fval`, `exitflag`
- `output`, `final_results`, `nodes`, `elements_geom`
- `figure_handle`, `macro_path`, `onshape_sketch_path`
- `output_path`（更新した unified vehicle artifact JSON）

## 実装上の注意

- `examples/run_frame_optimization.m` と EasyStart はこの関数を呼ぶだけの薄い入口です。
- canonical flow は rear suspension hardpoints と vehicle mass を要求し、足りない場合は早い段階で止まります。
- `VehicleParamsFile` を与えた場合は、その artifact 自体を更新します。fallback runtime artifact は「常に mirror される共通出口」ではありません。
