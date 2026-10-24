# tests/verify_data_processing.py

## 役割
Task 3 のチェックポイントとして、実 TTC データを使ったローダ・前処理・荷重グルーピングが想定どおり動くかを手順付きで確認する CLI スクリプトです。

## 入力と出力
- 入力: DriveBrake / Cornering の TTC CSV
- 出力: DataFrame 形状、列一覧、FZ・速度・温度範囲、フィルタ件数、荷重群検証ログ
- 生成物: なし

## 主な処理
- `verify_drivebrake_data()` は読込、`FZ` 正値化確認、速度・荷重・`SA`・温度フィルタ、`FX` の IQR 外れ値除去、荷重群境界確認まで行います。
- `verify_cornering_data()` は Cornering に対して荷重フィルタ、`FY` 外れ値除去、荷重群境界確認を行います。
- `main()` は両方を実行し、成功時に Task 4 へ進める旨を表示します。

## 存在理由
- pytest というより現場向けの逐次確認ツールで、前処理のどの段階で件数が減ったかを人間が把握しやすくするためにあります。
