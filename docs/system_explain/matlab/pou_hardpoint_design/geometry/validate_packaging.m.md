# `matlab/pou_hardpoint_design/geometry/validate_packaging.m`

## 1. 目的

最適化後の設計変数が最低限のパッケージ制約を満たすかを事後検証する関数です。rim クリアランス、キャリパクリアランス、ベルクランクの左右位置、高さを確認します。

## 2. 入出力と依存関係

- 入力:
  - `design_vars = [Xm, Ym, Zm, Xb, Yb, Zb]`
  - `geom`
  - `delta_range`
- 出力:
  - `valid`: 違反なしなら `true`
  - `violations`: 違反文言のセル配列
- 依存関係:
  - `solve_upright_position_3d`
  - `norm`, `sprintf`

## 3. 全体処理フロー

1. 設計変数を M ピボットとベルクランクへ分けます。
2. M ピボットの upright 局所半径 `sqrt(Xm^2 + Zm^2)` を使って、rim 内半径との余裕を調べます。
3. 各操舵角で M ピボットを chassis 座標へ変換し、キャリパ中心からのクリアランスを調べます。
4. ベルクランクの Y 座標が wheel center より inboard 側かを左右別ルールで確認します。
5. ベルクランクの Z 座標が地面に近すぎないか、高すぎないかを見ます。
6. 違反が 1 件も無ければ `valid=true` とします。
7. 違反がある場合はコンソールへも一覧表示します。

## 4. 関数詳細

### `validate_packaging(design_vars, geom, delta_range)`

- **rim 判定**  
  ループしていますが、実計算に `delta` は使っていません。`M_pivot_upright` の半径だけで最小余裕を求めます。

- **caliper 判定**  
  こちらは操舵角ごとに M ピボットの chassis 座標が変わるため、`solve_upright_position_3d` を毎回呼びます。

- **左右別 inboard 判定**  
  左輪は `y_bellcrank < y_wheel` だと outboard 扱い、右輪は `y_bellcrank > y_wheel` だと outboard 扱いです。

- **高さ判定**  
  SAE の Z down を前提に、`z_bellcrank > 0.20` を地面に近すぎ、`z_bellcrank < -0.40` を高すぎとしています。

- **ローカル関数**  
  このファイルにローカル関数はありません。
