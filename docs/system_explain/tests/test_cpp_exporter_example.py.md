# tests/test_cpp_exporter_example.py

## 役割
車両シミュレーション側で使う C++ ヘッダをどう生成するかを、現実的なパラメータ例で示すデモ兼テストです。

## 守っている振る舞い
- `export_cpp_header()` に Magic Formula の縦力係数と TMeasy の横力係数を渡すと、`namespace fsae_tire` を持つヘッダが生成されること。
- 生成ヘッダに `ModelType`、`MagicFormulaFxParams`、`TMeasyFyParams`、`HybridTireModel` が含まれること。
- 波括弧数と `#ifndef/#define/#endif` の存在で、最低限の構文整合が取れていること。

## 主なフィクスチャ/ヘルパー
- fixture はなく、テスト内で FX/FY のサンプル辞書を直接定義します。
- `tempfile.TemporaryDirectory()` を使い、一時ヘッダを生成します。

## 重要なアサーション・生成物
- テストは生成ヘッダ全文を標準出力へ表示するため、手動確認にも使えます。
- 生成物は `hoosier_tire_params.h` です。
- 実コンパイルはせず、ドキュメント的な利用例を保ちながら出力内容だけを固定します。
