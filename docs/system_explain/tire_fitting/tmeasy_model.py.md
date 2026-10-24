# `tire_fitting/tmeasy_model.py`

## 役割

旧系パイプラインで使う TMeasy 定常力モデルです。  
縦力・横力を配列対応で計算し、荷重依存摩擦、飽和関数、キャンバスラストを含む簡略 TMeasy を提供します。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - `slip_ratio` または `slip_angle`
  - `fz`
  - `camber_angle`
  - `mu_*_peak`, `k_mu_*`, `Kx`, `Ky`, `C_gamma`
- 主な出力:
  - `Fx` / `Fy` のスカラーまたは `numpy.ndarray`

## 主要クラス

### `TMeasyModel`

#### `__init__(fz_nominal=1000.0)`
- 荷重依存摩擦で使う公称荷重を保持します。

#### `_compute_load_dependent_friction(fz, mu_peak, k_mu)`
- `mu = mu_peak * (1 - k_mu * (Fz - Fz_nom) / Fz_nom)` を返します。

#### `_compute_saturation_function(s)`
- `s<1` では放物線、`s>=1` では 1 に飽和する関数です。

#### `compute_longitudinal_force(slip_ratio, fz, mu_x_peak, k_mu_x, Kx)`
- 正規化変形 `xi_bar` から縦力を計算します。
- `FZ==0` を含む場合はゼロ力を返します。

#### `compute_lateral_force(slip_angle, fz, camber_angle, mu_y_peak, k_mu_y, Ky, C_gamma=0.0)`
- `eta_bar` による横力と `C_gamma * camber * Fz` のキャンバ寄与を合成します。

## データフロー

1. 入力を `np.atleast_1d` でベクトル化します。
2. 荷重依存摩擦を計算します。
3. 正規化変形 → 飽和関数 → 符号復元の順で力へ変換します。
4. 横力では最後にキャンバ寄与を加えます。

## システム内での位置づけ

`main.py`、`optimizer.TireParameterOptimizer`、`validate_predictions.py` が使う **旧 TMeasy 計算核** です。  
`tire_fitting/models/tmeasy.py` は新しい共通インタフェース版で、こちらは後方互換寄りの独立実装です。
