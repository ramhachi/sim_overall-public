# getRequiredSizeClass.m
## ファイル目的
- 規則パラメータから、部材グループに要求される FSAE サイズ区分を取得する。

## 実行フロー
- 既定値を `'None'` に設定する。
- `reg_params.element_groups.<group_name>.required_size` を順に探索する。

## 関数詳細
### `size_class = getRequiredSizeClass(group_name, reg_params)`
- シグネチャ: `function size_class = getRequiredSizeClass(group_name, reg_params)`
- 入力:
  - `group_name`: 例 `main_hoop`
  - `reg_params`: 規則 JSON 由来 struct
- 出力:
  - `size_class`: `'A'/'B'/'C'/'D'/'None'`
- 責務:
  - グループ別必須サイズの安全な参照
- 主要ロジック:
  - `isfield` を段階的に使い、未定義時は `'None'`
- 依存関係:
  - なし
- 前提条件・注意事項:
  - 規則 struct の構造が不完全でも例外を出さず既定値で返す
