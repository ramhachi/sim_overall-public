# tests/test_cpp_export_integration.py

## 役割
ハイブリッド設定の YAML 出力から C++ ヘッダ生成までが一貫して動くことを確認する統合テストです。

## 守っている振る舞い
- `test_yaml_to_cpp_export_workflow()` は `HybridConfigData` を YAML に保存し、それを再読込して C++ ヘッダへ変換できることを確認します。
- 同テストは include guard、`namespace tire_model`、`enum class ModelType`、`MagicFormulaFxParams`、`TMeasyFyParams`、`HybridTireModel`、各数値パラメータ文字列が出力に入ることを検証します。
- `test_cpp_header_syntax_validity()` は波括弧数、header guard 数、namespace の閉じ方、`};` の数を数えて生成ヘッダの構造整合性を見ます。

## 主なフィクスチャ/ヘルパー
- fixture は使わず、テスト内で `ModelConfig` と `FittingMetrics` を構築します。
- `tempfile.TemporaryDirectory()` に `tire_config.yaml` と `tire_params.h` を作ります。

## 重要なアサーション・生成物
- 数値の文字列表現まで比較しており、エクスポータが値を落とさないことを保証します。
- 生成物は一時 YAML と一時 C++ ヘッダのみです。
- スキップはなく、出力構造の契約を直接固定するテストです。
