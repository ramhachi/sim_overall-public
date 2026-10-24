# `tire_fitting/models/magic_formula.py`

## 役割

hybrid 系で使う軽量 Pacejka Magic Formula 実装です。  
完全版 MF5.2 ではなく、このリポジトリの最適化・出力系で必要な係数群だけに絞った簡略モデルです。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - `slip_ratio` / `slip_angle`
  - `fz`
  - `params` 辞書
  - 任意の `temperature`, `camber_angle`
- 主な出力:
  - `calculate_fx()` / `calculate_fy()` の力値

## 主要クラス

### `MagicFormulaModel`

#### `__init__(params=None)`
- モデル係数辞書を保持します。

#### `calculate_fx(slip_ratio, fz, **kwargs)`
- `pDx1`, `pCx1`, `pEx1`, `pKx1-3`, `pDx2_temp`, `T_ref`, `fz0` を使って縦力を返します。
- 温度補正は `mu_x = pDx1 + pDx2_temp * (T - T_ref)` です。

#### `calculate_fy(slip_angle, fz, **kwargs)`
- `pDy1`, `pCy1`, `pEy1`, `pKy1-3`, `pKy_cam`, `fz0` を使って横力を返します。
- 最後にキャンバ寄与を加えます。

#### `get_model_type()` / `get_parameters()` / `set_parameters(params)`
- hybrid 共通インタフェース向けの識別・取得・更新 API です。

#### `_magic_formula(slip, B, C, D, E)`
- `D * sin(C * atan(...))` 形のコア式です。

## データフロー

1. `fz0` 基準で荷重差 `dfz` を作ります。
2. 係数辞書から `B, C, D, E` を組み立てます。
3. `_magic_formula()` で基本力を計算し、必要なら温度 / キャンバ補正を加えます。

## システム内での位置づけ

hybrid パイプラインの比較対象モデルです。  
`ModelFitter` と `comparison_report.py` が `TireModel` インタフェース経由で呼び出します。
