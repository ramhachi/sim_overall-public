# `tire_fitting/models/hybrid.py`

## 役割

縦力用モデルと横力用モデルを別々に差し込める合成モデルです。  
TMeasy と Magic Formula を方向別に組み合わせても、外からは `TireModel` として扱えます。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - `fx_model: TireModel`
  - `fy_model: TireModel`
  - 力計算時のスリップ量、荷重、追加引数
- 主な出力:
  - `calculate_fx()` / `calculate_fy()` の結果
  - サブモデル別パラメータ辞書
- 例外:
  - `TypeError`: `TireModel` 実装でないモデルを渡した場合

## 主要クラス

### `HybridTireModel`

#### `__init__(fx_model, fy_model)`
- 両サブモデルが `TireModel` 実装かを検査して保持します。

#### `calculate_fx(slip_ratio, fz, **kwargs)`
- 縦力だけ `fx_model` へ委譲します。

#### `calculate_fy(slip_angle, fz, **kwargs)`
- 横力だけ `fy_model` へ委譲します。

#### `get_model_type()`
- `hybrid(fx=..., fy=...)` 形式で構成を返します。

#### `get_parameters()`
- `fx_model`, `fx_params`, `fy_model`, `fy_params` を返します。

#### `set_parameters(params)`
- `fx_params` / `fy_params` があれば各サブモデルへ部分更新を流します。

## データフロー

1. 初期化時に方向別サブモデルを受け取ります。
2. 呼ばれた方向だけ該当サブモデルへ委譲します。
3. 構成情報やパラメータ取得も 2 モデル分まとめて返します。

## システム内での位置づけ

hybrid 設計の **実体表現** です。  
モデル選択結果を 1 つのオブジェクトへまとめたい場面で使える、方向分離型ラッパになっています。
