# `tire_fitting/models/tmeasy.py`

## 役割

`TireModel` インタフェースに合わせた新しい TMeasy 実装です。  
旧 `tmeasy_model.py` の式を保ちながら、hybrid 系で扱えるよう `calculate_fx()` / `calculate_fy()` とパラメータ管理 API を追加しています。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - `fz_nominal`
  - `params` 辞書
  - スリップ量、荷重、キャンバ角
- 主な出力:
  - 縦力 / 横力
  - 現在パラメータ辞書

## 主要クラス

### `TMeasyModel`

#### `__init__(fz_nominal=1000.0, params=None)`
- 公称荷重とモデル係数辞書を保持します。

#### `calculate_fx(slip_ratio, fz, **kwargs)`
- `params` から `mu_x_peak`, `k_mu_x`, `Kx` を取り出し、内部の `compute_longitudinal_force()` を呼びます。

#### `calculate_fy(slip_angle, fz, **kwargs)`
- `params` から `mu_y_peak`, `k_mu_y`, `Ky`, `C_gamma` を取り出し、`camber_angle` を反映して `compute_lateral_force()` を呼びます。

#### `get_model_type()` / `get_parameters()` / `set_parameters(params)`
- hybrid 共通インタフェース向けの識別・取得・更新 API です。

#### `compute_longitudinal_force(...)` / `compute_lateral_force(...)`
- 数式本体は旧 `tmeasy_model.py` とほぼ同じです。
- ベクトル入力に対応し、`FZ==0` ではゼロ力を返します。

## データフロー

1. 公開 API ではまず `validate_inputs()` で `fz` を確認します。
2. 内部パラメータ辞書から必要係数を取り出します。
3. 旧 TMeasy 数式群で実際の力を計算します。

## システム内での位置づけ

hybrid 系の **TMeasy アダプタ兼実装本体** です。  
`ModelFitter`、`HybridTireModel`、comparison report で共通利用され、旧 `tmeasy_model.py` よりインタフェース指向になっています。
