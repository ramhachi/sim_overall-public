# frame_optimize/matlab/beam_fem_solver_tcp_java.m

## 役割

Java Socket を使って TCP FEM サーバへ接続する **標準クライアント**です。通常運用の frame objective はこのファイルを通して C++ サーバを呼びます。

## 主な処理

- `'start'`, `'stop'`, `'shutdown'` などの制御コマンドを受ける
- 解析時は接続失敗に応じて再試行する
- main process では必要時にサーバ起動を許可し、parallel worker では待機再試行を優先する
- `SOLVE` JSON を送り、1 行応答を MATLAB struct に戻す

## 実装上の注意

- サーバ管理は `beam_fem_server_manager.m` に委譲します。
- ポートは `get_fem_server_host_port.m` で解決します。
- canonical flow では solver backend の正規窓口です。
