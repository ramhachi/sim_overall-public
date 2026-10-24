# `matlab/pou_hardpoint_design/geometry/solve_upright_position_3d.m`

## 1. 目的

upright の 3D 位置姿勢を操舵角から求める関数です。現行の主経路では、上下ボールジョイントで定義されたキングピン軸まわりの純回転として扱います。

## 2. 入出力と依存関係

- 入力:
  - `delta`: 操舵角 [rad]
  - `geom`: `define_suspension_geometry` の出力
- 出力 `upright_state`:
  - `R_upright`, `t_upright`
  - `U_U`, `L_U`, `TR_U`
  - `kingpin_dir`, `position`, `rotation`
  - `upper_arm_upright_chassis`, `lower_arm_upright_chassis`, `steer_angle`
- 依存関係:
  - `axis_angle_rotation`
  - ローカル関数群（後述）

## 3. 全体処理フロー

1. `geom` から nominal の upper/lower/tie-rod upright 点とキングピン軸を取得します。
2. `axis_angle_rotation(kp_dir, delta)` でキングピン回転行列 `R_kp` を作ります。
3. `t_up = kp_origin - R_kp * kp_origin` として、キングピン原点を不動点にする剛体変換を作ります。
4. nominal 点群へこの剛体変換を適用し、新しい `U_U`, `L_U`, `TR_U` を得ます。
5. 変換と点群を `upright_state` に詰めて返します。

## 4. 主関数詳細

### `solve_upright_position_3d(delta, geom)`

- コメントにもある通り、現行ロジックは「pure kingpin kinematics」です。A アーム角や tie-rod 長を未知数として解く一般ソルバは使っていません。
- `kingpin_dir` は回転軸そのものなので、返却時も固定の `kp_dir` をそのまま入れています。

## 5. ローカル関数詳細

### `eval_res_3(x, geom, U0, L0, TR0, L_kingpin, L_tie_rod, L_TR_U)`

3 変数残差関数です。`x = [theta_u, theta_l, phi]` を受け、キングピン長、tie-rod 長、upright 剛性を満たすかを 3 本の残差で返します。現行主経路からは呼ばれていませんが、より一般的な幾何ソルバの名残です。

### `rotate_point_around_axis(p, axis, axis_origin, angle)`

点 `p` を任意軸まわりに回す補助関数です。軸原点へ平行移動し、`axis_angle_rotation` を掛け、元座標へ戻します。

### `rotation_between_vectors(a, b)`

ベクトル `a` を `b` に重ねる回転行列を返します。平行時は単位行列、反平行時は `get_perpendicular(a)` を軸に 180 度回転、それ以外は外積ベースの式を使います。

### `estimate_arm_angle(p_target, p_nominal, arm_axis, arm_origin)`

アーム軸まわりの角度を初期推定する関数です。軸直交成分同士のなす角を符号付きで返します。現行主経路からは呼ばれていません。

### `get_perpendicular(v)`

入力ベクトルへ直交する単位ベクトルを 1 本返します。X 軸に近すぎるかどうかで外積相手を切り替えます。
