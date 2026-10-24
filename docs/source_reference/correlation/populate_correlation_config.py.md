# correlation/populate_correlation_config.py

## 役割
実車相関設定 JSON の記入率を確認し、テンプレートを埋めたシミュレータ用 `params_correlated.json` を生成します。

## 主な関数
- `is_filled()`, `count_fields()`, `check_config()`
- `field()`, `assign_if_filled()`, `deg_list_to_rad()`
- `export_params_run()`
- `main()`

## 背景
実車計測値は未記入項目が混在しやすいため、必須/任意の充足率を見ながらテンプレートへ安全に反映する必要があります。

## 使用ライブラリ
- `json`, `argparse`, `math`, `pathlib`

## パイプライン上の位置
実車相関ワークフローの設定整備段です。
