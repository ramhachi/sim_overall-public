# tests/analyze_v8_results.py

## 役割
`|SA| < 5deg` を入れた v8 前処理後でも縦力パラメータが下限に張り付く理由を、実データ分布と TMeasy 予測誤差から確認する単発分析スクリプトです。

## 入力と出力
- 入力: `data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv`
- 出力: SR/FZ/FX/摩擦係数統計、`mu_x_peak` スイープ時 RMSE
- 生成物: `output/v8_data_analysis.png`

## 主な処理
- `TireDataLoader` で読み込んだ DriveBrake から `SR/FX/FZ/SA` 欠損を除き、`FZ>0` と `|SA|<5deg` を満たす行だけ残します。
- `|FX|/FZ` の分布と 95/99 パーセンタイルを出し、荷重帯ごとのピーク摩擦係数も計算します。
- 固定パラメータ `mu_x_peak=1.30, k_mu_x=0.05, Kx=15000` で RMSE を出し、さらに `mu_x_peak` を 1.5〜2.5 に振って比較します。

## プロットと存在理由
- `SR vs FX`、`SR vs |FX|/FZ`、摩擦係数ヒストグラム、予測残差を 2x2 で保存します。
- 「下限値が本当に最良か、データが高 μ を支持しているか」を人手で切り分けるための診断資料です。
