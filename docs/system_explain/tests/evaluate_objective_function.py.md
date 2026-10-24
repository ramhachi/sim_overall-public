# tests/evaluate_objective_function.py

## 役割
複数の縦力パラメータ候補で目的関数を直接評価し、下限解が最良なのか、最適化器が悪いのかを切り分けるスクリプトです。

## 入力と出力
- 入力: `data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv`
- 出力: 各候補の目的関数値、RMSE、最大誤差、結論ログ
- 生成物: `output/objective_function_evaluation.png`

## 主な処理
- `load_and_filter_data()` は `TireDataPreprocessor` で速度・荷重・温度・`max_sa_for_longitudinal=0.5deg` を適用し、さらに `FX` の IQR 外れ値除去を行います。
- `evaluate_objective()` は `TMeasyModel.compute_longitudinal_force()` で予測し、残差二乗和・RMSE・最大誤差・予測範囲を返します。
- `main()` は「下限 / 中央値 / 上限 / データ駆動推定」の 4 候補を比較し、最小目的関数の候補を採用して結論を出します。

## プロットと存在理由
- 目的関数比較、RMSE 比較、最良候補の予測対実測、現行下限候補の予測対実測を並べます。
- 目的関数面の形を人間が確認し、前処理起因か探索起因かを判断するための解析です。
