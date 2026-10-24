# `matlab/pou_hardpoint_design/test_bounds.m`

## 1. 目的

各ホイールの設計変数境界と初期値が妥当かを手動確認する診断スクリプトです。`define_suspension_geometry` が返す M ピボット境界・ベルクランク境界を一覧表示します。

## 2. 入出力と依存関係

- 形式: スクリプト
- 入力: なし（内部で `wheel_idx = 0:3` を走査）
- 出力: 各変数の下限・上限・初期値・判定結果を標準出力へ表示
- 依存関係: `addpath('geometry')`, `define_suspension_geometry`

## 3. 全体処理フロー

1. コンソールを初期化し、`geometry` を `addpath` します。
2. `wheel_names = {'FL', 'FR', 'RL', 'RR'}` を用意します。
3. 各ホイールで `geom = define_suspension_geometry(wheel_idx)` を取得します。
4. `lb`, `ub`, `x0` を 6 変数分まとめます。
5. `Xm, Ym, Zm, Xb, Yb, Zb` ごとに `lb < ub` と `lb <= x0 <= ub` を確認します。
6. 1 輪ごとに総合結果を出し、最後に終了メッセージを表示します。

## 4. ブロック詳細

- **境界構築**  
  `lb = [geom.M_pivot_bounds.lower; geom.bellcrank_bounds.lower]`、`ub = [...]` という本番最適化と同じ構成を使っています。

- **個別判定**  
  `lb(i) >= ub(i)` ならエラー、初期値が外なら警告、それ以外は `OK` と表示します。

- **ローカル関数**  
  このファイルにローカル関数はありません。
