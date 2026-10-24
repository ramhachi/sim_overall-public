# computeTubeSectionProperties.m
## ファイル目的
- 円管/角管の断面積・断面二次モーメント・ねじり定数・せん断面積を計算する。

## 実行フロー
- 入力値と板厚妥当性を検証する。
- 内径/内辺長を計算する。
- `shape` に応じて round/square の公式を適用する。
- 出力 struct へまとめる。

## 関数詳細
### `props = computeTubeSectionProperties(D_outer, t, shape)`
- シグネチャ: `function props = computeTubeSectionProperties(D_outer, t, shape)`
- 入力:
  - `D_outer`: 外径または外辺長 [mm]
  - `t`: 板厚 [mm]
  - `shape`: `'round'` または `'square'`
- 出力:
  - `props`: `A_mm2`, `Iy_mm4`, `Iz_mm4`, `J_mm4`, `Asy_mm2`, `Asz_mm2`
- 責務:
  - FEM と規則判定で使う断面特性計算
- 主要ロジック:
  - round: 厳密円環公式、`J = 2I`
  - square: 薄肉近似のねじり定数式を使用
- 依存関係:
  - なし
- 前提条件・注意事項:
  - `t < D_outer/2` が必須
  - 角管ねじり定数は近似式
