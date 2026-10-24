# checkFrameCompliance.m
## ファイル目的
- フレーム構造が FSAE 2026 の管サイズ・材料・必要構造要件を満たすか評価する。

## 実行フロー
- 出力 `report` を初期化する。
- 必要なら `reg_params` をファイルパスから読み込む。
- 要素グループごとにサイズ要件を確認する。
- 材料制約とアルミ板厚要件を確認する。
- 必須構造 `main_hoop`, `front_hoop` の存在を確認し総合判定する。

## 関数詳細
### `[pass, penalties, report] = checkFrameCompliance(frame_struct, catalog, materials, reg_params)`
- シグネチャ: `function [pass, penalties, report] = checkFrameCompliance(frame_struct, catalog, materials, reg_params)`
- 入力:
  - `frame_struct.nodes`
  - `frame_struct.elements`
  - `frame_struct.element_groups`
  - `catalog`, `materials`, `reg_params`
- 出力:
  - `pass`: 総合適合
  - `penalties`: 総ペナルティ
  - `report`: 違反詳細
- 責務:
  - 規則適合性の集約チェック
- 主要ロジック:
  - サイズ不足は不足段数二乗に比例して罰則
  - アルミは `t<3.0 mm` を保守的に不適合扱い
  - 必須グループ欠如は 50,000 罰則
- 依存関係:
  - `loadRegulationParams`
  - `getRequiredSizeClass`
  - `computeTubeSectionProperties`
  - `determineSizeClass`
  - `checkMaterialRestriction`
  - ローカル関数 `isSizeCompliant`
- 前提条件・注意事項:
  - 材料判定は `E_GPa > 150` を鋼とみなす近似
  - `checkMaterialRestriction` 呼び出し引数順が本来実装と整合しないため、コード追従時は注意

### `is_compliant = isSizeCompliant(actual_size, required_size)`
- シグネチャ: `function is_compliant = isSizeCompliant(actual_size, required_size)`
- 入力:
  - `actual_size`, `required_size`
- 出力:
  - `is_compliant`
- 責務:
  - サイズ階層比較
- 主要ロジック:
  - `None < D < C < B < A` の数値化比較
- 依存関係:
  - `containers.Map`
- 前提条件・注意事項:
  - 未知サイズは false
