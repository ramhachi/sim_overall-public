# beam_fem_solver_tcp.m
## ファイル目的
- `tcpclient` を使って常駐 FEM TCP サーバへ解析要求を送る。

## 実行フロー
- `start/stop/shutdown` コマンドを受ける場合はサーバ管理だけ行う。
- 初回呼び出し時に `beam_fem_server_manager('start')` でサーバを起動する。
- `SOLVE` リクエスト JSON を送信する。
- 1 行 JSON 応答を読み、`result` 構造体へ変換する。

## 関数詳細
### `result = beam_fem_solver_tcp(nodes, elements, bcs, loads, options)`
- シグネチャ: `function result = beam_fem_solver_tcp(nodes, elements, bcs, loads, options)`
- 入力:
  - 解析時: `nodes`, `elements`, `bcs`, `loads`, 任意 `options`
  - 制御時: `nodes='start'|'stop'|'shutdown'`
- 出力:
  - 解析結果または状態メッセージ
- 責務:
  - TCP ベースの解析要求送受信
- 主要ロジック:
  - persistent `server_handle/server_ready` を保持
  - 毎回接続を張り直す設計
- 依存関係:
  - `beam_fem_server_manager`
  - `get_fem_server_host_port`
  - `force2D`
- 前提条件・注意事項:
  - 応答空文字や送信失敗は即 `error`

### `out_cell = force2D(mat)`
- シグネチャ: `function out_cell = force2D(mat)`
- 入力: `mat`
- 出力: JSON 互換 2 次元表現
- 責務:
  - 1 行配列整形
- 主要ロジック:
  - 1 行だけ cell 化
- 依存関係: なし
- 前提条件・注意事項:
  - `beam_fem_solver` と同等実装
