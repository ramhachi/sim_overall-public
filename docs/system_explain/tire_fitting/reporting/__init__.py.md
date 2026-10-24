# `tire_fitting/reporting/__init__.py`

## 役割

reporting パッケージの公開窓口です。  
比較レポート生成機能の最小 API だけを再公開します。

## 実行と入出力

- 直接実行するファイルではありません。
- 入力:
  - `comparison_report.py` の公開要素
- 出力:
  - `generate_comparison_report`
  - `ComparisonReportData`

## 主要要素

### 再公開シンボル
- `generate_comparison_report`
- `ComparisonReportData`

## データフロー

1. 下位モジュールから必要シンボルを import します。
2. `__all__` で公開面を固定します。

## システム内での位置づけ

実装本体を持たない **glue ファイル** です。  
hybrid レポート機能の import を単純化するために置かれています。
