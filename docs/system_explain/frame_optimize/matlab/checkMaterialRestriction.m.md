# checkMaterialRestriction.m
## ファイル目的
- 部材グループと材料組合せが規則上許容されるかを判定する。

## 実行フロー
- `steel_only_groups` を確認し、鋼限定部位か判定する。
- 複合材禁止グループを確認する。
- 非鋼材についてアルミ最小板厚条件を確認する。

## 関数詳細
### `[is_valid, reason] = checkMaterialRestriction(group_name, pipe_data, material_data, reg_params)`
- シグネチャ: `function [is_valid, reason] = checkMaterialRestriction(group_name, pipe_data, material_data, reg_params)`
- 入力:
  - `group_name`: 部材グループ名
  - `pipe_data`: 管外径・板厚を含む pipe struct
  - `material_data`: 対応材料 struct
  - `reg_params`: 規則 struct
- 出力:
  - `is_valid`: 適合可否
  - `reason`: 不適合理由
- 責務:
  - 材料選定のルールチェック
- 主要ロジック:
  - `is_steel` / `is_composite` を用いた制約判定
  - アルミは welded 条件を優先し、`min_thickness_welded_mm` と規則既定値の最大を採用
- 依存関係:
  - なし
- 前提条件・注意事項:
  - 非鋼＝即アルミ扱いに近い簡略化がある
  - 溶接前提で保守的に判定する
