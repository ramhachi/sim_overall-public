# loadMaterials.m
## ファイル目的
- `materials.json` を MATLAB の field 参照しやすい材料 struct へ変換する。

## 実行フロー
- 入力数・型を検証する。
- JSON を読み込み、`materials` フィールドを検証する。
- 各材料の必須項目を確認する。
- `id` を field 名へ変換して `materials.<id>` に格納する。

## 関数詳細
### `materials = loadMaterials(json_path)`
- シグネチャ: `function materials = loadMaterials(json_path)`
- 入力:
  - `json_path`: 材料 JSON パス
- 出力:
  - `materials`: `material_id` を field に持つ struct
- 責務:
  - 材料データの読み込みと整形
- 主要ロジック:
  - `E_GPa`, `G_GPa`, `rho_kg_m3`, `Sy_welded_MPa`, `Su_welded_MPa` を必須化
  - cell/struct 配列両対応
- 依存関係:
  - `jsondecode`, `fileread`
  - `matlab.lang.makeValidName`
- 前提条件・注意事項:
  - 不正 JSON や必須項目欠落は `error`
