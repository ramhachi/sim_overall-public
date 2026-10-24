# aero_map_4d.m

## 目的
`aero_map_4d` は、車速・前後ライドハイト・ロール角・ヨー角から空力の力とモーメントを返す関数です。戻り値は `Fx`、`Fy`、`Fz`、`Mx`、`My`、`Mz` と、前後ダウンフォース配分 `Fz_front`、`Fz_rear` を含む構造体です。

コード冒頭のコメントでは「4次元エアロダイナミクスマップ」「C++実装と完全一致」と説明されています。実装上はグリッド配列を定義したうえで `compute_coefficients` を呼びますが、係数計算はテーブル補間ではなく式ベースで行われます。

## 入口とシグネチャ
```matlab
function result = aero_map_4d(velocity, rh_front_mm, rh_rear_mm, roll_rad, yaw_rad, params)
```

### 入力
- `velocity`: 車速 `[m/s]`
- `rh_front_mm`: フロントライドハイト `[mm]`
- `rh_rear_mm`: リアライドハイト `[mm]`
- `roll_rad`: ロール角 `[rad]`
- `yaw_rad`: ヨー角、コメント上は車体スリップ角 `[rad]`
- `params`: 空力パラメータ構造体。省略時は空構造体から既定値を補完

### 出力
`result` 構造体。
- `Fx`: 抗力。コード上は `-q * A_ref * CD`
- `Fy`: サイドフォース
- `Fz`: リフト。負値がダウンフォース
- `Mx`: ロールモーメント。実装では常に `0`
- `My`: ピッチモーメント
- `Mz`: ヨーモーメント
- `Fz_front`: 前輪側ダウンフォース配分
- `Fz_rear`: 後輪側ダウンフォース配分

## 外部依存
外部ファイル呼び出しはありません。同一ファイル内ローカル関数 `compute_coefficients` のみを使います。

## 処理の流れ
1. `params` が無い場合は空構造体を作成します。
2. `rho`、`A_ref`、`L_ref`、`x_cg`、`CL_default`、`CD_default`、`CoP_default` を `isfield` で補完します。
3. グリッドとして `rh_f`、`rh_r`、`roll`、`yaw` を定義します。
4. `compute_coefficients(...)` で `CL`、`CD`、`CY`、`CoP_x`、`CoP_dist` を得ます。
5. 動圧 `q = 0.5 * rho * velocity^2` を計算します。
6. `Fx`、`Fy`、`Fz` を係数から計算します。
7. `lever_arm = CoP_x - x_cg` で重心から圧力中心までの距離を求め、`My = Fz * lever_arm` を計算します。
8. `abs(yaw_rad) > 0.01` のときだけ `Mz = Fy * lever_arm` を与え、それ以下では `0` にします。
9. `Mx` は計算せず常に `0` とします。
10. `CoP_dist` で `Fz_front` と `Fz_rear` に前後配分します。

## 既定値と定数
`params` の既定値は次の通りです。
- `rho = 1.225`
- `A_ref = 1.0`
- `L_ref = 1.6`
- `x_cg = 0.8`
- `CL_default = -2.5`
- `CD_default = 1.0`
- `CoP_default = 0.45`

グリッド定義は次の通りです。
- 前ライドハイト: `[20, 30, 40, 50, 60] mm`
- 後ライドハイト: `[30, 40, 50, 60, 70, 80] mm`
- ロール: `[-5, -2.5, 0, 2.5, 5] deg`
- ヨー: `[-15, -10, -5, 0, 5, 10, 15] deg`

ただし `compute_coefficients` 内ではこの `grid` 構造体の中身は参照されません。コード上は受け渡しのみで、実際の係数は下記の式で直接作られます。

## ローカル関数 `compute_coefficients`
```matlab
function coef = compute_coefficients(rh_f, rh_r, roll, yaw, grid, params)
```

### 役割
空力係数と圧力中心情報を返します。

### 詳細処理
- 基本値として `CL_default`、`CD_default`、`CoP_default` を使います。
- ライドハイト効果:
  ```matlab
  rh_effect = 1.0 + 0.015 * (40 - rh_f) + 0.010 * (55 - rh_r)
  ```
  を `0.6` 以上 `1.4` 以下に制限します。前後ライドハイトが低いほど `rh_effect` は大きくなります。
- ロール効果:
  ```matlab
  roll_loss = 1.0 - 3.0 * abs(roll)
  ```
  を `0.7` 以上 `1.0` 以下に制限します。絶対ロール角が大きいほど損失が増えます。
- ヨー効果:
  ```matlab
  yaw_effect = cos(yaw * 2)
  ```
  を `0.6` 以上 `1.0` 以下に制限します。
- 横力係数は
  ```matlab
  CY_val = sin(yaw * 2) * 0.5
  ```
  です。
- 最終係数は次の式です。
  - `CL = CL_base * rh_effect * roll_loss * yaw_effect`
  - `CD = CD_base * (1.0 + 2.0 * yaw^2)`
  - `CY = CY_val`
- 圧力中心 `CoP_x` は `CoP_default` から前後ライドハイトで補正します。
  - `rh_f` が大きいと `+ (rh_f - 40) * 0.003`
  - `rh_r` が大きいと `- (rh_r - 55) * 0.002`
- 前後配分 `CoP_dist` は
  ```matlab
  dist = 0.5 + (rh_f - rh_r + 15) * 0.003
  ```
  を `0.35` から `0.65` に制限します。

### 注意点
- `grid` 引数は未使用です。
- `L_ref` は既定値設定だけで、このファイル内では使われません。

## 仮定とフォールバック
- `yaw_rad` が `0.01 rad` 以下なら `Mz = 0` に固定します。
- ロールモーメント `Mx` は簡易化されており常に `0` です。
- ライドハイト、ロール、ヨーの影響はすべて上下限付きで飽和します。

## 保存物・描画
この関数単体では保存処理も描画処理もありません。
