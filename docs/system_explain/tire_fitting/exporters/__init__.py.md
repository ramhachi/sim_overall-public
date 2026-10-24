# `tire_fitting/exporters/__init__.py`

## 役割

hybrid 用 exporter パッケージの公開窓口です。  
YAML exporter と C++ header exporter の主要 API を再公開します。

## 実行と入出力

- 直接実行するファイルではありません。
- 入力:
  - `yaml_exporter.py`, `cpp_exporter.py` の公開関数 / dataclass
- 出力:
  - `from tire_fitting.exporters import ...` で使える記号

## 主要要素

### 再公開シンボル
- `export_hybrid_config`
- `import_hybrid_config`
- `HybridConfigData`
- `ModelConfig`
- `FittingMetrics`
- `export_cpp_header`
- `export_cpp_header_from_yaml_data`

## データフロー

1. 下位モジュールから必要関数と dataclass を import します。
2. `__all__` で公開面を固定します。

## システム内での位置づけ

本体ロジックは持たない **glue 層** です。  
hybrid 出力機能の import 経路を短く保つためのパッケージ初期化ファイルです。
