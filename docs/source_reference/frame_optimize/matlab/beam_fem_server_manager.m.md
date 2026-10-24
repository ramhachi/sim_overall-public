# frame_optimize/matlab/beam_fem_server_manager.m

## 役割

TCP FEM サーバの **ライフサイクル管理器**です。ローカル実行バイナリと Docker 実行の両方を扱い、起動・停止・疎通確認・ログ表示を担います。

## 主要アクション

- `start`: サーバ起動
- `stop`: サーバ停止
- `check_alive`: `PING` による疎通確認
- `print_logs`: 直近ログの表示
- `describe`: 現在のハンドル情報を要約

## 実装上の注意

- 起動時は既存プロセス掃除と readiness 待機を行います。
- local バイナリが使えなければ Docker を試します。
- canonical flow 自体は `beam_fem_solver_tcp_java('start'/'stop')` 経由でこの管理器を使います。
