# beam_fem_solver_tcp_java.m
## ファイル目的
- Java Socket API で低レイテンシに FEM TCP サーバへ接続する高速クライアント。

## 実行フロー
- 制御コマンドならサーバ起動/停止とソケット解放を行う。
- 解析時は最大 2 回まで接続再試行する。
- 必要に応じ既存サーバ確認または新規起動を行う。
- `SOLVE` JSON を送信し、1 行応答を受信して解析結果へ変換する。

## 関数詳細
### `result = beam_fem_solver_tcp_java(nodes, elements, bcs, loads, options)`
- シグネチャ: `function result = beam_fem_solver_tcp_java(nodes, elements, bcs, loads, options)`
- 入力:
  - 解析入力または `'start'/'stop'/'shutdown'`
- 出力:
  - 解析結果または状態メッセージ
- 責務:
  - 高速 TCP クライアント
- 主要ロジック:
  - parallel worker では自力起動せず待機再試行を優先
  - main プロセスは接続失敗時に server state を落として再起動を許容
  - 送信前に `request.command='SOLVE'`
- 依存関係:
  - `beam_fem_server_manager`
  - `getCurrentTask`
  - `get_fem_server_host_port`
  - `force2D`
- 前提条件・注意事項:
  - デバッグ用 `fprintf('Sending request...')` が毎回出る
  - サーバ応答が `null` の場合はログ出力を促す

### `out_cell = force2D(mat)`
- シグネチャ: `function out_cell = force2D(mat)`
- 入力: `mat`
- 出力: JSON 互換配列
- 責務:
  - 1 行配列整形
- 主要ロジック:
  - 1 行のみ cell 化
- 依存関係: なし
- 前提条件・注意事項:
  - 他ソルバラッパと同一方針
