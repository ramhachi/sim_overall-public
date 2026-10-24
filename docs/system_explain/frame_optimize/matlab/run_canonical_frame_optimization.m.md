# `frame_optimize/matlab/run_canonical_frame_optimization.m`

## 役割

現行の canonical frame optimization API です。EasyStart と example wrapper の両方から呼ばれ、**artifact 解決から保存まで**を一つの関数に閉じ込めます。

## 流れ

1. 入力 artifact を解決し、suspension hardpoints を fail-fast で確認する
2. geometry / `K_chassis` を取り込み、penalty 設定と cockpit zone を組む
3. `custom_mixed_ga` で `frameObjectiveFunction` を探索する
4. 最良解を再生成し、viewer / PNG / 任意の SolidWorks macro / Onshape 3D sketch FeatureScript を出す
5. `save_frame_optimization_artifact` で `params.frame` と `K_chassis` を更新した unified artifact を保存する
6. `VehicleParamsFile` があればその artifact を更新し、無ければ fallback runtime artifact を作る

## 重要なオプション

- `VehicleParamsFile`
- `MassTargetKg`
- `GenerateSolidWorksMacro`
- `GenerateOnshape3DSketch`
- `WaitForFigureClose`
- `PopulationSize`, `MaxGenerations`, `UseParallel`

## システム上の位置づけ

スクリプトから本体を切り離して、base workspace 依存を減らした中心点です。
