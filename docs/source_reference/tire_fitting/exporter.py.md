# tire_fitting/exporter.py

## 役割
旧来 TMeasy フィット結果を YAML と C++ ヘッダへ書き出すエクスポータです。

## 主なクラス
- `ParameterExporter`

## 主なメソッド
- `export_yaml()`: 人間可読な TMeasy パラメータ YAML を生成
- `export_cpp_header()`: `static constexpr` ベースの C++ 構造体を生成

## 背景
シミュレータ実装は C++、調整系は Python という構成なので、同定結果の受け渡しフォーマットが必要です。本ファイルは旧フォーマット互換を維持する役目も持ちます。

## 使用ライブラリ
- `yaml`, `pathlib`, `datetime`, `logging`, `numpy`

## 入出力
- 入力: TMeasy パラメータ辞書と評価指標
- 出力: YAML、`.h/.hpp`

## パイプライン上の位置
旧 TMeasy 単独フローの最終出力段です。
