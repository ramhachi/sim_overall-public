# tests/unit/test_cpp_exporter.py

## 役割
ハイブリッドタイヤモデル向け C++ ヘッダ生成器が、識別子整形・構造体生成・完全出力・YAML 経由変換を正しく行うかを確認します。

## 守っている振る舞い
- `_sanitize_name()` が空白や記号を C++ 識別子向け大文字名へ正規化すること。
- TMeasy/Magic Formula 各方向の構造体生成関数が、必要係数を `double` 初期値として埋め込むこと。
- `HybridTireModel` 構造体が Fx/Fy のモデル種別とパラメータ型を正しく保持すること。
- `export_cpp_header()` が header guard、namespace、enum、各 struct、ハイブリッド設定を含むファイルを生成すること。
- Fx/Fy とも TMeasy、または Magic Formula のような同種構成も扱えること。
- 不正モデル種別や YAML 必須節欠落で明確に `ValueError` になること。
- `namespace` 上書きや `export_cpp_header_from_yaml_data()` も期待どおり動くこと。

## 主なフィクスチャ/ヘルパー
- `TestSanitizeName`、`TestStructGeneration`、`TestCppHeaderExport`、`TestExportFromYamlData` の 4 クラスに責務が分かれています。
- 各テストは一時ディレクトリと辞書パラメータをその場で組み立てます。

## アサーションの意味
文字列包含確認は、生成物が人間可読であるだけでなく C++ 側コード生成の入力として十分な構造を持つことを見ています。失敗系の検証は、不完全な YAML や誤指定モデルが静かに壊れたヘッダを生まないための安全装置です。
