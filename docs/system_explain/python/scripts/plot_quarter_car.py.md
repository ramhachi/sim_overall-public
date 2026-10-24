# `python/scripts/plot_quarter_car.py`

## 1. 役割

Quarter Car シミュレーション結果を **変位・速度・力・ダンパヒステリシス** の 4 観点で可視化するスクリプトです。

## 2. 入出力

- 実行例: `python3 python/scripts/plot_quarter_car.py <result.csv> [output_dir]`
- 入力: `time`, `z_s`, `z_u`, `z_road`, `dz_s`, `dz_u`, `F_spring`, `F_damper`, `F_tire`
- 出力:
  - `quarter_car_plot.png`
  - 標準出力の要約

## 3. 制御フロー

1. `main()` が CSV と出力先を決定します。
2. `load_data()` が CSV を読みます。
3. `print_summary()` がピーク値と最終状態を出します。
4. `plot_quarter_car()` が 2x2 の図を作成して保存・表示します。

## 4. 関数

### `load_data(csv_path: str) -> pd.DataFrame`
- CSV を読み込んで返します。
- 副作用: 件数表示。

### `plot_quarter_car(df, output_dir=None)`
- 役割: 4 パネル図を描画します。
- パネル:
  - ばね上/ばね下/路面変位
  - ばね上/ばね下速度
  - ばね・ダンパ・タイヤ力
  - 相対速度に対するダンパ力の F-V ループ
- 特記事項: F-V ループは時間で色分けされます。
- 副作用: PNG 保存、`plt.show()`
- 戻り値: `Figure`

### `print_summary(df)`
- 役割: 時間長、ピーク変位、最大速度、最大ダンパ力、最終変位を表示します。

### `main()`
- 役割: CLI 入口です。
- デフォルトパスは `/workspace/...` のためコンテナ実行を想定しています。
