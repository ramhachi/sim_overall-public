# `matlab/grid_search_pid.m`

## 1. ファイルの目的

主要な PID ゲインのみを粗い格子点で探索し、ベイズ最適化より短時間で良好な候補を見つける関数です。

## 2. 主な入出力または依存関係

- 入力:
  - `track_type`
  - `track_radius`
  - `sim_time`
- 出力: `best_gains`
- 依存関係:
  - Docker / `cpp/laptime_sim`
  - `visualize_laptime_sim`
  - MATLAB の `readtable`, `figure`

## 3. 処理の流れ / アルゴリズム

1. トラック種別から C++ 実行引数を決めます。
2. `Kp_steer`, `Kd_steer`, `Kh_steer` の候補集合を総当たりします。
3. 各組み合わせで Docker 内 `laptime_sim` を走らせ、一時 CSV を MATLAB 側へコピーします。
4. 横偏差 RMS、ヘディング RMS、最大横偏差、ステア滑らかさからコストを計算します。
5. 全結果から最小コストを選び、ヒートマップや散布図を描画します。
6. 最良ゲインで最終シミュレーションを再実行し、結果テキストも保存します。

## 4. ファイル内の関数

### `grid_search_pid(track_type, track_radius, sim_time)`

- 本ファイル唯一の関数です。
- `Ki_steer`, `Kff_steer`, `Kp_speed`, `Ki_speed` は固定し、探索次元を減らしています。
- 評価失敗時は単にその試行を捨て、成功試行だけ `results` に蓄積します。
- 可視化では `Kh_steer` の中央値断面で `Kp` vs `Kd` のコストマップを作ります。
- 最後に `grid_search_results_<track>.txt` を出力し、CLI 引数相当の値も残します。

## 5. 注意点・前提条件

- `project_root = fileparts(pwd)` なので、想定実行位置は `matlab/` ディレクトリです。
- 一時 CSV 名は固定 (`grid_temp.csv`) のため、同時並行実行には向きません。
- 失敗試行は詳細原因を保持せず「unstable」として扱います。
