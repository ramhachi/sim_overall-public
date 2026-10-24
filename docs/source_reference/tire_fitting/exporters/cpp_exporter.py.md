# tire_fitting/exporters/cpp_exporter.py

## 役割
ハイブリッドタイヤモデルを C++ 向けヘッダへ変換します。

## 主な関数
- `_sanitize_name()`
- `_generate_tmeasy_*_struct()`
- `_generate_magic_formula_*_struct()`
- `_generate_hybrid_model_struct()`
- `export_cpp_header()`

## 背景
C++ 側ランタイムで縦横別モデルを再構成できるよう、モデル種別 enum と方向別パラメータ struct を生成します。

## 使用ライブラリ
- `pathlib`, `datetime`, `re`

## 入出力
- 入力: 縦横モデル種別、パラメータ辞書、タイヤ名
- 出力: C++ ヘッダ

## パイプライン上の位置
Python 同定結果を C++ シミュレータへ橋渡しする最終コード生成段です。
