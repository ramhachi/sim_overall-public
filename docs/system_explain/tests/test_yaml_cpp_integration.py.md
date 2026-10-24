# tests/test_yaml_cpp_integration.py

## 役割
YAML エクスポータと C++ ヘッダエクスポータを連携させ、片方で出した内容がもう片方でも失われないことを確認する統合テストです。

## 守っている振る舞い
- `test_yaml_cpp_round_trip()` は hybrid 設定を YAML に出し、再読込後に C++ ヘッダへ変換し、モデル種別と代表パラメータが保たれることを確認します。
- `test_both_exporters_with_tmeasy_only()` は両方向とも TMeasy の場合に `ModelType::TMEASY` と `TMeasyFxParams/TMeasyFyParams` が出ることを見ます。
- `test_both_exporters_with_magic_formula_only()` は両方向とも Magic Formula の場合に `ModelType::MAGIC_FORMULA` と対応 struct が出ることを見ます。

## 主なフィクスチャ/ヘルパー
- fixture は使わず、各テスト内で `HybridConfigData`、`ModelConfig`、`FittingMetrics` を組みます。
- `tempfile.TemporaryDirectory()` に YAML と HPP を作ります。

## 重要なアサーション・生成物
- YAML の `longitudinal_force` / `lateral_force` 構造と、C++ 側の `fx_model_type` / `fy_model_type` が一致することが中心です。
- 生成物は一時 `config.yaml` / `params.h` 群です。
- skip はなく、モデル混在・単独の 3 パターンを固定しています。
