# `matlab/pou_hardpoint_design/optimization/generate_initial_guess.m`

## 1. 目的

`fmincon` の初期値を戦略別に作る関数です。設計変数の境界内で nominal / conservative / aggressive / random の候補を返します。

## 2. 入出力と依存関係

- 入力:
  - `geom`
  - `target_poly`
  - `strategy`（省略時 `nominal`）
- 出力:
  - `x0`: 6×1 初期値 `[Xm, Ym, Zm, Xb, Yb, Zb]`
- 依存関係: `rand`, `max`, `min`

## 3. 全体処理フロー

1. `geom` から上下限を組みます。
2. `switch strategy` で初期値生成法を分けます。
3. 生成後に `x0 = max(lb, min(ub, x0))` で必ず境界内へクリップします。

## 4. 関数詳細

### `generate_initial_guess(geom, target_poly, strategy)`

- **`nominal`**  
  `geom.M_pivot_initial` と `geom.bellcrank_initial` をそのまま使います。

- **`conservative`**  
  M ピボットを小さめオフセット、ベルクランクを境界中央へ置き、実現可能性優先の初期値を作ります。

- **`aggressive`**  
  `abs(target_poly.jack_c1)` を見て `Xm` を少し大きめにします。`Zm` は `lb(3)+0.02` と `-0.04` の比較で低すぎないよう制限します。

- **`random`**  
  各成分を上下限の一様乱数で選びます。

- **未知戦略**  
  既知でない文字列なら `error('Unknown strategy: ...')` を投げます。

- **ローカル関数**  
  このファイルにローカル関数はありません。
