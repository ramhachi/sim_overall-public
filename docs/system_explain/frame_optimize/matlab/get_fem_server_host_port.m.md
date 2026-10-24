# get_fem_server_host_port.m
## ファイル目的
- FEM TCP サーバのホスト側ポート番号を環境変数から安全に取得する。

## 実行フロー
- `FSAE_HOST_FEM_PORT` を読み、未設定または不正値なら既定値 `50051` を使う。

## 関数詳細
### `port = get_fem_server_host_port()`
- シグネチャ: `function port = get_fem_server_host_port()`
- 入力: なし
- 出力: `port`
- 責務:
  - 標準ポートの公開
- 主要ロジック:
  - 実処理は `parse_port_env` に委譲
- 依存関係:
  - `getenv`
- 前提条件・注意事項:
  - ホスト名は固定で `localhost`

### `port = parse_port_env(env_name, default_port)`
- シグネチャ: `function port = parse_port_env(env_name, default_port)`
- 入力:
  - `env_name`: 環境変数名
  - `default_port`: 既定値
- 出力:
  - `port`: 正の整数ポート
- 責務:
  - 文字列→整数の安全変換
- 主要ロジック:
  - 空文字、非数、非整数、0 以下を既定値へフォールバック
- 依存関係:
  - `str2double`, `getenv`
- 前提条件・注意事項:
  - 範囲上限チェックは未実装
