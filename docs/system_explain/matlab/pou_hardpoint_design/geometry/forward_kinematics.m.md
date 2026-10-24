# `matlab/pou_hardpoint_design/geometry/forward_kinematics.m`

## 1. 目的

設計変数からジャッキアップ曲線・キャンバー曲線・pushrod 長を計算する順運動学関数です。POU 最適化の目的関数と評価指標の基礎になっています。

## 2. 入出力と依存関係

- 入力:
  - `design_vars = [Xm, Ym, Zm, Xb, Yb, Zb]`
  - `geom`
  - `delta_range`: 操舵角配列 [rad]
- 出力:
  - `jack_curve`: pushrod 変位 [m]
  - `camber_curve`: キャンバー角 [rad]
  - `pushrod_lengths`: 絶対 pushrod 長 [m]
- 依存関係:
  - `solve_upright_position_3d`
  - `norm`, `asin`

## 3. 全体処理フロー

1. 設計変数の前半を `M_pivot_upright`、後半を `bellcrank_pivot` に分けます。
2. 0 操舵の upright 状態を求め、基準 pushrod 長 `L_pushrod_nominal` を計算します。
3. 左輪なら `spindle_nominal=[0;-1;0]`、右輪なら `[0;1;0]` として基準スピンドル方向を決めます。
4. 各操舵角で `solve_upright_position_3d` を呼び、M ピボットを chassis 座標へ変換します。
5. ベルクランク点までの距離を pushrod 長として保存します。
6. `-(L_pushrod - L_pushrod_nominal)` をジャッキアップとして保存します。つまり pushrod が短くなると正になります。
7. 回転後スピンドル方向の Z 成分から `asin` でキャンバー角を求めます。

## 4. 関数詳細

### `forward_kinematics(design_vars, geom, delta_range)`

- **ジャッキアップ定義**  
  コメント通り「compression = shorter pushrod」を正としているため、名目長より短いと正値です。

- **キャンバー定義**  
  実装はスピンドル外向きベクトルの Z 成分だけを使います。コメントでは「上端が外へ倒れる正キャンバー」を説明しています。

- **ローカル関数**  
  このファイルにローカル関数はありません。
