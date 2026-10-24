# `tire_fitting/models/base.py`

## 役割

hybrid モデル群の共通インタフェースを定義する基底モジュールです。  
縦力・横力計算、パラメータ取得更新、入力検証をモデル実装へ強制します。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - スリップ比、スリップ角、`fz`
  - モデル固有パラメータ辞書
- 主な出力:
  - 抽象メソッド契約
  - `TireForces(fx, fy)`
- 例外:
  - `validate_inputs()` が `ValueError` を送出

## 主要クラス

### `TireForces`
- `fx`, `fy` をまとめる単純 dataclass です。

### `TireModel`
- 抽象基底クラスです。

#### 抽象メソッド
- `calculate_fx(slip_ratio, fz, **kwargs)`
- `calculate_fy(slip_angle, fz, **kwargs)`
- `get_model_type()`
- `get_parameters()`
- `set_parameters(params)`

#### `calculate_forces(slip_ratio, slip_angle, fz, **kwargs)`
- `calculate_fx()` と `calculate_fy()` をまとめて呼ぶ便宜メソッドです。

#### `validate_inputs(fz, param_name='fz')`
- 荷重が有限かつ正かを検査します。

## データフロー

1. 派生クラスが抽象 API を実装します。
2. 共通ヘルパ `calculate_forces()` と `validate_inputs()` を再利用します。

## システム内での位置づけ

hybrid モデル設計の **契約定義** です。  
`ModelFitter`、`HybridTireModel`、comparison report がモデル種別を意識せず扱える前提をここで提供します。
