# `tire_fitting/exporters/cpp_exporter.py`

## 役割

hybrid タイヤ設定を C++ ヘッダへ変換する exporter です。  
縦力モデルと横力モデルが別種でも表現できるよう、モデル種別 enum と方向別 parameter struct を生成します。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - `fx_model_type`, `fy_model_type`
  - 各方向のパラメータ辞書
  - タイヤ名、出力パス、namespace
- 主な出力:
  - 1 枚の C++ ヘッダ
- 副作用:
  - 親ディレクトリ作成
- 例外:
  - `ValueError`: 未知のモデル種別、YAML 構造不足
  - `PermissionError`: 書込不可

## 主要関数

### `_sanitize_name(name)`
- タイヤ名を header guard 用識別子へ正規化します。

### `_generate_tmeasy_fx_struct(...)` / `_generate_tmeasy_fy_struct(...)`
- TMeasy 用の方向別 struct を文字列で生成します。

### `_generate_magic_formula_fx_struct(...)` / `_generate_magic_formula_fy_struct(...)`
- Magic Formula 用の方向別 struct を生成します。

### `_generate_hybrid_model_struct(fx_model_type, fx_params_struct, fy_model_type, fy_params_struct, indent='    ')`
- `ModelType` enum 値と方向別 parameter struct を束ねた `HybridTireModel` struct を作ります。

### `export_cpp_header(...)`
- メイン API です。
- include guard、namespace、enum、方向別 parameter struct、hybrid struct を組み立てて保存します。

### `export_cpp_header_from_yaml_data(yaml_data, filepath, namespace='tire_model')`
- YAML 読込済み辞書から必要項目を抜き出し、`export_cpp_header()` に委譲します。

## データフロー

1. モデル種別妥当性を確認します。
2. タイヤ名から header guard を生成します。
3. 方向別 model type に応じた struct 文字列を選びます。
4. 1 枚のヘッダへ連結して書き出します。

## システム内での位置づけ

hybrid パイプラインの **C++ 連携出口** です。  
`main_hybrid.py` から直接使われ、YAML と同じ選択結果を C++ 側へ持ち込むための形式変換を担います。
