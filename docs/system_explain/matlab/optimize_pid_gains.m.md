# `matlab/optimize_pid_gains.m`

## 1. ファイルの目的

ラップタイムシミュレータの PID ドライバ用ゲインを Bayesian Optimization で自動調整する関数です。

## 2. 主な入出力または依存関係

- 入力:
  - `track_type`
  - `track_radius`
  - `sim_time`
  - `initial_speed`
  - `n_iter`
  - `params_file`
- 出力: `results` (`bayesopt` の結果オブジェクト)
- 依存関係:
  - Docker / `cpp/laptime_sim`
  - `visualize_laptime_sim`
  - `bayesopt`, `optimizableVariable`

## 3. 処理の流れ / アルゴリズム

1. トラック種別と初期速度などを正規化し、必要なら `--params_file` 引数を作ります。
2. OS に応じて Docker 実行ファイルの PATH を補います。
3. コンテナが未起動なら `docker compose up -d` で立ち上げます。
4. トラックに応じて探索範囲と初期シード `initial_x` を決めます。
5. `objective_function()` が各候補ゲインで `laptime_sim` を実行し、走行完遂性・横偏差・方位誤差・速度誤差・滑らかさから cost を計算します。
6. `bayesopt()` で最小コスト点を探索し、最良ゲインで最終シミュレーションと可視化・テキスト保存を行います。

## 4. ファイル内の定義

### `optimize_pid_gains(track_type, track_radius, sim_time, initial_speed, n_iter, params_file)`

- 本ファイルの公開関数であり、残りのロジックはその内部に閉じています。
- `ASE_autocross` だけは `brake_gain_ratio` と `corner_thr_red` も最適化対象に追加します。
- バッチモードでは `PlotFcn` を無効化し、対話 GUI を使わずに回せるようにしています。

### `objective_function(params)`（ネスト関数）

- 候補ゲインを CLI 引数へ変換し、Docker 内で `laptime_sim` を実行します。
- 出力 CSV から有効行のみ抽出し、ジャンプや大きすぎる横偏差を DNF 相当として扱います。
- 完走性が低い場合は距離不足ペナルティを強く課し、完走時は RMS 誤差・滑らかさ・平均速度不足を組み合わせたコストを返します。

## 5. 注意点・前提条件

- 実行位置は `matlab/` 想定で、出力一時 CSV 名は固定 `opt_temp.csv` です。
- 評価関数は CSV 列名 (`lateral_error`, `heading_error_deg`, `speed_error`, `s` など) に依存します。
- DNF 判定や重み係数はトラック種別に強く依存した経験則です。
