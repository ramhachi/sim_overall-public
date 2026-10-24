# python/scripts/plot_quarter_car.py

## 役割
クォーターカー結果を変位・速度・力・ダンパヒステリシスとして可視化します。

## 主な関数
- `load_data()`
- `plot_quarter_car()`: `z_s`/`z_u`、速度、`F_spring`/`F_damper`/`F_tire`、F-V ループを描画
- `print_summary()`: 最大変位や最大ダンパ力を表示
- `main()`

## 背景・理論
1 輪モデルではサスペンション単体の縦応答確認が主目的です。Bouc-Wen ダンパの履歴特性を F-V ループで観察できる構成になっています。

## 使用ライブラリ
- `pandas`, `numpy`, `matplotlib`

## 入出力
- 入力: クォーターカー結果 CSV
- 出力: 端末要約、`quarter_car_plot.png`

## パイプライン上の位置
低次元検証モデルの結果確認に使う可視化ユーティリティです。
