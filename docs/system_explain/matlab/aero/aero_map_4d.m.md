# `matlab/aero/aero_map_4d.m`

## 役割

このファイルは、前後ライドハイト・ロール・ヨーを入力として空力係数と作用点情報を返す 4 次元エアロマップの MATLAB クラスです。  
クラス名は `aero_map_4d` で、`handle` 継承のため参照型として動作します。

## クラス入口

```matlab
obj = aero_map_4d()
```

コンストラクタは引数なしで、デフォルトグリッド初期化とサンプルデータ生成を自動実行します。

## プロパティ

### 定数的に初期化されるプロパティ

- `rho = 1.225`
- `A_ref = 1.0`
- `L_ref = 1.6`
- `x_cg = 0.8`
- `CL_default = -2.5`
- `CD_default = 1.0`
- `CoP_default = 0.45`

`L_ref` はこのファイル内では宣言されるだけで、計算には使っていません。

### グリッド

- `grid_rh_f`
- `grid_rh_r`
- `grid_roll`
- `grid_yaw`

### データ配列

- `CL_data`
- `CD_data`
- `CY_data`
- `CoP_x_data`
- `CoP_dist_data`

## メソッド

### コンストラクタ `aero_map_4d()`

次の 2 つを順に呼びます。

1. `init_default_grid()`
2. `init_sample_data()`

そのため、生成直後から補間可能なデータを持ちます。

### `init_default_grid(obj)`

4 次元グリッドとデータ配列の形を定義します。

#### グリッド値

- フロントライドハイト: `[20, 30, 40, 50, 60]` mm
- リアライドハイト: `[30, 40, 50, 60, 70, 80]` mm
- ロール: `deg2rad([-5, -2.5, 0, 2.5, 5])`
- ヨー: `deg2rad([-15, -10, -5, 0, 5, 10, 15])`

#### 配列初期化

4 次元サイズ `n` を作り、

- `CL_data = zeros(n)`
- `CD_data = zeros(n)`
- `CY_data = zeros(n)`
- `CoP_x_data = zeros(n)`
- `CoP_dist_data = 0.5 * ones(n)`

で初期化します。

### `init_sample_data(obj)`

全グリッド点を 4 重ループで走査し、サンプル係数を埋めます。

#### 基準値

- `CL_base = CL_default`
- `CD_base = CD_default`
- `CoP_base = CoP_default`

#### ライドハイト効果

```matlab
rh_effect = 1 + 0.015 * (40 - rh_f) + 0.010 * (55 - rh_r)
```

で計算し、`0.6` から `1.4` にクリップします。

#### ロール効果

```matlab
roll_loss = 1 - 3 * abs(roll)
```

で計算し、`0.7` から `1.0` にクリップします。  
`roll` はラジアンなので、係数低下量もラジアン依存です。

#### ヨー効果

- `yaw_effect = cos(yaw * 2)` を `0.6` から `1.0` にクリップ
- `CY_val = sin(yaw * 2) * 0.5`

です。

#### 係数

- `CL_data = CL_base * rh_effect * roll_loss * yaw_effect`
- `CD_data = CD_base * (1 + 2 * yaw^2)`
- `CY_data = CY_val`

です。

#### 圧力中心

```matlab
CoP = CoP_base + (rh_f - 40) * 0.003 - (rh_r - 55) * 0.002
```

として `CoP_x_data` へ格納します。

#### 前後配分

```matlab
dist = 0.5 + (rh_f - rh_r + 15) * 0.003
```

を `0.35` から `0.65` にクリップし、`CoP_dist_data` へ保存します。

### `get_coeffs(obj, rh_f_mm, rh_r_mm, roll_rad, yaw_rad)`

5 種類のマップを `interp4d` で補間し、構造体 `coef` として返します。

- `coef.CL`
- `coef.CD`
- `coef.CY`
- `coef.CoP_x`
- `coef.CoP_dist`

### `compute_forces(obj, velocity, rh_f_mm, rh_r_mm, roll_rad, yaw_rad)`

補間した係数から空力とモーメントを計算します。

#### 動圧

- `q = 0.5 * rho * velocity^2`

#### 力

- `Fx = -q * A_ref * CD`
- `Fy = q * A_ref * CY`
- `Fz = q * A_ref * CL`

です。`CL_default` が負なので、基準条件では `Fz` も負になります。

#### モーメント

- `lever = CoP_x - x_cg`
- `My = Fz * lever`
- `Mz = Fy * lever`

です。

#### 前後配分

- `Fz_front = Fz * CoP_dist`
- `Fz_rear = Fz * (1 - CoP_dist)`

を返します。

### `interp4d(obj, data, rh_f, rh_r, roll, yaw)`

`interpn` を使う 4 次元線形補間関数です。

補間前に各入力をグリッド最小値と最大値の間へ丸め込みます。

- `rh_f`
- `rh_r`
- `roll`
- `yaw`

のすべてが `max(min(...))` でクランプされるため、グリッド外入力でも外挿せず端値補間になります。

## 補足

- このクラスはサンプルデータを自動生成するため、外部ファイル読込みには依存しません。
- `L_ref` は定義されていますが、このファイルでは未使用です。
