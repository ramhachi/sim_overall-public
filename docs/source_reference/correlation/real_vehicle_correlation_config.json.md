# correlation/real_vehicle_correlation_config.json

## 役割
実車の質量、足回り、タイヤ、パワートレイン、空力などを「入力値 + 測定方法 + 必須性」で管理する相関用入力ファイルです。

## 背景
`populate_correlation_config.py` は本ファイルの `required/value/method` 構造を走査して準備状況を判定します。

## 入出力
- 入力: 手計測・CAD・実験で得た実車パラメータ
- 出力: チェック結果や `params_correlated.json` 生成元

## パイプライン上の位置
実車相関プロセスの一次情報源です。
