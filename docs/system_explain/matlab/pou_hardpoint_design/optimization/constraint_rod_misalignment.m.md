# `matlab/pou_hardpoint_design/optimization/constraint_rod_misalignment.m`

## 1. 目的

`fmincon` 用の非線形不等式制約を返す関数です。ロッドエンド角、pushrod 長、インストレーション比、upright 半径、inboard クリアランス、ベルクランクと A アームの離隔をまとめて評価します。

## 2. 入出力と依存関係

- 入力:
  - `design_vars = [Xm, Ym, Zm, Xb, Yb, Zb]`
  - `geom`
  - `delta_range`
- 出力:
  - `c`: 不等式制約ベクトル（`c <= 0` で可）
  - `ceq`: 常に空配列
- 依存関係:
  - `solve_upright_position_3d`
  - `norm`, `dot`, `acos`, `vecnorm`

## 3. 全体処理フロー

1. 0 操舵時の M ピボット位置と nominal pushrod 方向を求めます。
2. 各操舵角で pushrod 方向を計算し、nominal 方向との角度差を `rod_angles` に保存します。
3. 各操舵角の pushrod 長も保存します。
4. 最大ロッド角、最小/最大 pushrod 長、0 操舵時 IR を不等式へ変換します。
5. hub center 近傍からの M ピボット距離、タイヤ中心面より inboard に入っているか、ベルクランクが A アーム取付点群から十分離れているかも追加します。
6. `ceq = []` を返します。

## 4. 関数詳細

### `constraint_rod_misalignment(design_vars, geom, delta_range)`

実際に返す `c` の各要素は次の通りです。

1. `max(rod_angles) - geom.rod_end_angle_max`
2. `geom.pushrod_length_min - min(pushrod_lengths)`
3. `max(pushrod_lengths) - geom.pushrod_length_max`
4. `0.03 - IR`
5. `IR - 1.50`
6. `dist_to_wc - max_upright_radius`
7. `abs(M_pivot_nominal(2)) - (abs(geom.y_wheel) - min_inboard_clearance)`
8. `min_bellcrank_arm_clearance - min_dist_to_arm`

### 実装上の注意

- コメント冒頭では 3 種類の制約だけを説明していますが、実装は 8 個まで拡張されています。
- `c = zeros(6,1)` で初期化した後に `c(7)`, `c(8)` を代入しているため、MATLAB 配列拡張で最終的に 8 要素になります。
- `idx_zero` が `delta_range` に含まれない場合は nominal pushrod 長で IR を計算します。

- **ローカル関数**  
  このファイルにローカル関数はありません。
