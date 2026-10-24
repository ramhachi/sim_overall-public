# determineSizeClass.m
## ファイル目的
- 断面特性と寸法から FSAE 2026 のサイズ区分を返す。

## 実行フロー
- 入力値を検証する。
- Size A → D → B → C の順に閾値を評価する。
- いずれも満たさない場合は `None` を返す。

## 関数詳細
### `size_class = determineSizeClass(I_mm4, A_mm2, D_mm, t_mm)`
- シグネチャ: `function size_class = determineSizeClass(I_mm4, A_mm2, D_mm, t_mm)`
- 入力:
  - `I_mm4`, `A_mm2`, `D_mm`, `t_mm`
- 出力:
  - `size_class`: `'A'/'B'/'C'/'D'/'None'`
- 責務:
  - 規則しきい値によるサイズ分類
- 主要ロジック:
  - 各サイズで 4 条件すべて満たす必要がある
  - Size D は大径条件を持つため B/C より先に判定
- 依存関係:
  - なし
- 前提条件・注意事項:
  - コメント上の例と閾値は FSAE 2026 F.3.4.1 を参照
  - 関数内の Size D 条件は一般的な小径例と整合しない可能性があり、実装値を確認して使う必要がある
