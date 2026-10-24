# tests/unit/test_exporter.py

## 役割
`ParameterExporter` が YAML と C++ ヘッダを正常系・異常系ともに安全に出力できるかを確認します。

## 守っている振る舞い
- `export_yaml()` が parse 可能な YAML を作り、主要メタデータと各節へ正しい値を書き込むこと。
- `NaN` を含むパラメータでは `ValueError` を出して無効出力を拒否すること。
- `export_cpp_header()` が include guard、namespace、struct、各係数を含むヘッダを作ること。
- タイヤ名の特殊文字が C++ 安全な識別子へ変換されること。
- `inf` など無効係数は C++ 出力でも拒否されること。
- YAML の必須節と C++ の全パラメータが欠けずに出ること。

## 主なフィクスチャ/ヘルパー
- `setup_method()` で共通の `params` と `metrics` を用意します。
- 各テストは一時 YAML/HPP を作成して内容を読み戻します。

## アサーションの意味
成功系では「書けた」だけでなく「再利用できる構造で書けた」ことを検証します。失敗系では、不正係数が下流へ流出しないことを強制しています。
