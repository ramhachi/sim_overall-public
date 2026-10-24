# correlation/params_correlated_template.json

## 役割
実車相関後にシミュレータへ流し込むパラメータ JSON の雛形です。

## 主な内容
- `mass`, `suspension`, `steering`, `tire`, `aero`, `powertrain`, `brake`
- タイヤ緩和長、空力参照密度、LSD 諸元など実行寄りのキー名を採用

## 背景
実車相関入力の構造と、シミュレータ実行時の構造は一致しないため、中間テンプレートが必要です。

## パイプライン上の位置
実車相関からシミュレーション入力への変換先フォーマットです。
