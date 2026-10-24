# tests/property/test_properties_export.py

## 役割
`ParameterExporter` の YAML/C++ 出力が、広いパラメータ空間でも必須構造を保つことを property-based に確認します。

## 守っている振る舞い
- YAML 出力を再読込したとき、`tire_model`、基本メタデータ、`longitudinal`、`lateral`、`camber`、`fitting_info` が欠けないこと。
- 各節の値が入力パラメータ・指標に一致すること。
- C++ ヘッダに include guard、`fsae::tire` 名前空間、`struct`、`static constexpr double` メンバ、単位コメント、生成日コメントが入ること。
- 生成ヘッダが全入力パラメータを落とさず保持すること。

## 主なフィクスチャ/ヘルパー
- `setup_method()` で `ParameterExporter` を生成します。
- Hypothesis が TMeasy パラメータと評価指標を生成します。
- 一時 YAML/HPP を作成して内容を読み戻します。

## アサーションの意味
値一致アサーションは単なるファイル生成成功ではなく、シリアライズの完全性を見ています。構造・名前空間・コメント確認は、出力が後続ツールや C++ 側組み込みでそのまま使えることの保証です。
