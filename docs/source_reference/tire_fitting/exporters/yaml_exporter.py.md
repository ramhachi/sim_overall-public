# tire_fitting/exporters/yaml_exporter.py

## 役割
ハイブリッドタイヤモデル設定を YAML へ保存・再読込するモジュールです。

## 主な型・関数
- `FittingMetrics`
- `ModelConfig`
- `HybridConfigData`
- `export_hybrid_config()`
- `import_hybrid_config()`
- `_parse_hybrid_format()`
- `_parse_legacy_format()`

## 背景
新形式では縦力と横力で別モデルを持つため、旧 TMeasy 専用 YAML より情報量が多くなります。本ファイルは新旧フォーマット互換を維持しつつ、選択理由やデータ点数も保存します。

## 使用ライブラリ
- `yaml`, `dataclasses`, `numpy`, `datetime`, `pathlib`

## 入出力
- 入力: `HybridConfigData` または YAML ファイル
- 出力: YAML 文字列表現、`HybridConfigData`

## パイプライン上の位置
Python 同定結果を人間可読かつ再読込可能な中間成果物にする層です。
