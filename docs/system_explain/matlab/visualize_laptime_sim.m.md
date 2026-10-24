# `matlab/visualize_laptime_sim.m`

## 1. ファイルの目的

ラップタイムシミュレーションの CSV を読み込み、軌跡・速度・誤差・制御入力・時系列・g-g 線図をまとめて生成する総合可視化関数です。

## 2. 主な入出力または依存関係

- 入力:
  - `csv_filename`
  - `track_type`
  - `track_radius`
- 出力:
  - 3 枚の図
  - `<csv_base>_main.png`
  - `<csv_base>_timeseries.png`
  - `<csv_base>_gg.png`
- 依存関係:
  - `readtable`
  - ローカル関数 `generate_track_centerline`
  - CSV 内の `time`, `x`, `y`, `vx`, `vy`, `r`, `s`, `kappa`, `steering`, `throttle`, `brake`, `speed`, `target_speed`, `lateral_error`, `heading_error_deg`

## 3. 処理の流れ / アルゴリズム

1. CSV を読み込み、必要列を展開します。
2. `drs_active` 列があれば利用し、なければ全 false とします。
3. `generate_track_centerline()` で比較用中心線を作ります。
4. メイン図で軌跡、速度、横偏差、方位誤差、ステア、スロットル/ブレーキを表示します。
5. 統計量を標準出力へ出し、メイン図を保存します。
6. 2 枚目で速度成分・ヨーレート・弧長・曲率・速度誤差・横偏差を可視化します。
7. 3 枚目で数値微分から縦横加速度を求め、g-g 図と理論限界楕円を描きます。

## 4. ファイル内の全関数

### `visualize_laptime_sim(csv_filename, track_type, track_radius)`

- メイン関数です。
- バッチモードでは figure を非表示にして PNG 保存のみ行います。
- DRS 区間は軌跡上ではシアン点、速度図では背景ハイライトとして示します。
- g-g 図では `ax = gradient(vx)/gradient(t)`、`ay = gradient(vy)/gradient(t) + vx.*r` を使っています。

### `generate_track_centerline(track_type, radius)`

- 可視化背景用の中心線を生成するローカル関数です。
- `circle`, `figure_eight`, `skidpad`, `straight_corner`, `ASE_autocross` に対応します。
- `ASE_autocross` は `data/ASE_autocross_sim.csv` を直接読みます。
- 8 の字と skidpad は C++ トラックモデルに合わせる意図で専用ロジックを持ちます。

## 5. 注意点・前提条件

- 可視化は CSV 列名に強く依存します。列不足の旧 CSV では動かない可能性があります。
- `ASE_autocross` のトラック読込は `matlab/` からの相対パス前提です。
- g-g 図の限界線は固定値の近似であり、車両設定から自動計算していません。
