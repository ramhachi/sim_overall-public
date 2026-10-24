# tests/analyze_sa_anomaly.py

## 役割
DriveBrake のスリップ角 `SA` に現れる極端な異常値を洗い出し、単位変換ミスではなくラップアラウンドや累積角度の可能性が高いことを説明する分析スクリプトです。

## 入力と出力
- 入力: `data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv`
- 出力: 標準出力の統計、異常値上位、修正案
- 生成物: `output/sa_anomaly_analysis.png`, `output/sa_normalization_comparison.png`

## 主な処理
- `analyze_sa_distribution()` は SA の基本統計、百分位、`|SA|>10/45/90/180deg` 件数、極端値の詳細を表示し、6 分割プロットを保存します。
- `check_unit_conversion()` は生 CSV の `SA` と `TireDataLoader` 通過後の `SA` を比べ、度→ラジアン変換係数が正しいかを確認します。
- `propose_correction()` は `((SA+180)%360)-180` による正規化と `|SA|<5deg` フィルタを提案し、正規化前後の分布を比較します。

## プロットと存在理由
- 全範囲 / 狭範囲ヒストグラム、`SA vs SR`、`SA vs FX`、累積分布で異常値の広がりを見せます。
- 正規化比較図は、後続の純縦力抽出条件を見直す根拠を作るためのものです。
