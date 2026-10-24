# `matlab/pou_hardpoint_design/geometry/generate_physical_bellcrank.m`

## 1. 目的

数学モデル上の pushrod 終端点を、物理的なベルクランク三角形とダンパ取付点へ写像する補助関数です。3D 可視化でロッカー形状を見せるために使われます。

## 2. 入出力と依存関係

- 入力:
  - `x_pushrod_chassis`: pushrod がベルクランク側でつながる chassis 座標
  - `pushrod_vec_nominal`: M ピボットからその点へ向かうベクトル
  - `geom`: サスペンション幾何構造体
- 出力 `phys`:
  - `pushrod_pin`
  - `frame_pivot`
  - `damper_pin`
  - `damper_chassis_mount`
- 依存関係:
  - `cross`, `norm`, `sign`
  - `axang2rotm`

## 3. 全体処理フロー

1. `x_pushrod_chassis` をそのまま `pushrod_pin` とします。
2. `pushrod_vec_nominal` を正規化して pushrod 方向を得ます。
3. pushrod 方向に直交するベクトルを作り、inboard 側を向くよう符号を調整します。
4. その直交方向へ 50 mm だけずらした点を `frame_pivot` にします。
5. 回転軸 `rot_axis = cross(pushrod_dir, orthogonal_vec)` を求め、90 度の L 字ベルクランクとして `damper_pin` を作ります。
6. `damper_pin` からさらに 200 mm、車体内側かつ上側へ延ばした点を `damper_chassis_mount` にします。

## 4. 関数詳細

### `generate_physical_bellcrank(x_pushrod_chassis, pushrod_vec_nominal, geom)`

- **`geom` の扱い**  
  引数にはありますが、現実装では `geom` のフィールドは参照していません。

- **直交ベクトル生成**  
  pushrod がほぼ横向き (`abs(y) > 0.99`) なら Z 軸と外積し、それ以外は Y 軸との外積を使います。

- **inboard 強制**  
  直交ベクトルの Y 符号が pushrod pin と同じなら反転し、必ず車体中心側へ pivot が来るようにしています。

- **ベルクランク角**  
  `bellcrank_angle = 90 deg` 固定です。motion ratio もコメント上 1.0 を想定していますが、計算としては arm 長を同じ 50 mm に揃えるだけです。

- **ローカル関数**  
  このファイルにローカル関数はありません。
