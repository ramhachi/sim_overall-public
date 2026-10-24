# `tire_pinn/export/__init__.py`

## 1. 役割

`tire_pinn.export` サブパッケージの薄い公開入口です。  
現状は `CPPExporter` だけを再公開します。

## 2. 公開 API

- `CPPExporter`
  - 学習済みモデルを C++ ランタイム向け JSON 群へ書き出します。

## 3. システム内での位置付け

- `from tire_pinn.export import CPPExporter` を成立させるための集約層です。
- 実装本体は `cpp_exporter.py` にあります。

## 4. 入出力と副作用

- 入力: なし
- 出力: なし
- 副作用: import のみです。
