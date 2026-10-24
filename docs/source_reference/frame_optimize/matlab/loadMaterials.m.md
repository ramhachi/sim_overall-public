# frame_optimize/matlab/loadMaterials.m

## 役割

`materials.json` を読み込み、material id をキーにした MATLAB struct へ変換する loader です。

## 使われ方

- `run_canonical_frame_optimization.m`
- `frameObjectiveFunction.m`
- 各種テスト

## 実装上の注意

- pipe catalog の `material_id` から E, G, 密度を引く基盤データです。
