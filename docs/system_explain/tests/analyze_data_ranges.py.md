# tests/analyze_data_ranges.py

## 役割
実 TTC データの分布を見て、縦力用 `SA<0.5deg` と横力用 `SR<0.01` が厳しすぎないかを確認する分析スクリプトです。v3/v4 条件と緩和案 v5 を比較し、使えるデータ量がどこで失われているかを説明します。

## 入力と出力
- 入力: `data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv`, `data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv`
- 出力: 標準出力の統計ログ
- 生成物: `output/data_range_analysis_drivebrake.png`

## 主な処理
- `analyze_drivebrake_data()` は `SR/SA/FZ/FX/V/P` の範囲、`|SA|` と `|SR|` の閾値別件数、`SA-SR` の関係を集計します。
- 同関数は 2x2 図で SA ヒストグラム、SR ヒストグラム、`SA vs SR`、`|SA| vs |SR|` を描きます。
- `analyze_cornering_data()` は Cornering 側の `SA/FZ/FY` を確認し、`SR` 列があれば分布も出します。
- `main()` は緩和後の期待件数と SR 範囲を再計算し、データ量増加と高スリップ領域の回収可否を示します。

## プロットと存在理由
- 現行閾値と提案閾値を縦線で重ね、どの条件がボトルネックかを視覚化します。
- 最適化が下限値に張り付く前段として、十分な学習データが残っているかを診断するための補助解析です。
