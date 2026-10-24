# `frame_optimize/examples/test_tcp_simple.m`

## 役割

- `beam_fem_solver_tcp` の最小疎通確認用スクリプトです。
- 2 ノード・1 要素の最小ケースで、初回起動と再呼び出しの両方を確認します。

## スクリプトの流れ

- `matlab/` をパス追加します。
- 2 ノード・1 要素・片持ち梁に相当する最小データを作ります。
- 1 回目の `beam_fem_solver_tcp` 呼び出しでサーバ起動込みの時間を測ります。
- 2 回目の呼び出しで接続キャッシュ後の時間を測ります。
- 最後に `beam_fem_solver_tcp('shutdown')` で終了します。

## 入力

- `nodes`: 2x3
- `elements`: 1x9
- `bcs`: ノード1完全拘束
- `loads`: ノード2の Fx=1000

## 出力

- 実行時間
- `result.mass`
- `result.K_torsion`
- 最大変位

## 前提・注意点

- 最小機能確認用であり、ねじり剛性値そのものに物理的意味は薄いです。
- TCP サーバの自動起動・接続維持の挙動確認が主目的です。
