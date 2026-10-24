# beam_fem_server_manager.m
## ファイル目的
- ローカル実行または Docker 実行の FEM TCP サーバを起動・停止・診断するライフサイクル管理器。

## 実行フロー
- 外部 API `beam_fem_server_manager(action, ...)` が `start/stop/check_alive/print_logs/describe` を振り分ける。
- `start` は既存サーバ掃除後、OS に応じて local と Docker を優先順付きで試行する。
- 起動後は `PING` で readiness を待つ。
- `stop` は shutdown request、PID kill、Docker cleanup を組み合わせて停止する。

## 関数詳細
### `varargout = beam_fem_server_manager(action, varargin)`
- シグネチャ: `function varargout = beam_fem_server_manager(action, varargin)`
- 入力: `action`, 任意 `handle`
- 出力: action に応じた handle/状態/説明
- 責務: 公開 API
- 主要ロジック: action ごとに local 関数へ委譲
- 依存関係: 下記全ローカル関数
- 前提条件・注意事項: 未知 action は `UnknownAction`

### `handle = get_optional_handle(args)`
- シグネチャ: `function handle = get_optional_handle(args)`
- 入力: 可変引数 cell
- 出力: 先頭 handle または空
- 責務: 引数正規化
- 主要ロジック: 空なら `[]`
- 依存関係: なし
- 注意: 補助用途のみ

### `handle = start_server_only()`
- シグネチャ: `function handle = start_server_only()`
- 入力: なし
- 出力: サーバ handle
- 責務: 起動シーケンス全体
- 主要ロジック:
  - `cleanup_existing_server`
  - 既存外部サーバが生きていれば external handle を返す
  - OS 別優先順で `try_start_local/try_start_docker`
- 依存関係: `get_runtime_context`, `check_server_alive`
- 注意: すべて失敗すると詳細をまとめて `StartFailed`

### `[handle, err_msg] = try_start_local(ctx)`
- シグネチャ: `function [handle, err_msg] = try_start_local(ctx)`
- 入力: 実行環境情報 `ctx`
- 出力: handle, 診断メッセージ
- 責務: ローカルバイナリ起動
- 主要ロジック:
  - ネイティブ実行可能ファイル探索
  - 見つかれば `start_local_server` と `wait_for_server_ready`
  - 非互換バイナリ検出時は説明付きでスキップ
- 依存関係: `find_local_server_binary`, `find_present_candidates`
- 注意: Windows/macOS で説明文が分岐

### `[handle, err_msg] = try_start_docker(ctx)`
- シグネチャ: `function [handle, err_msg] = try_start_docker(ctx)`
- 入力: `ctx`
- 出力: handle, 診断メッセージ
- 責務: Docker 起動経路
- 主要ロジック:
  - `detect_docker`
  - 必要時 `try_autostart_docker_desktop`
  - `start_docker_server` 後に readiness 待機
- 依存関係: `detect_docker`, `try_autostart_docker_desktop`
- 注意: Docker 非可用時は例外でなく err_msg を返す

### `wait_for_server_ready(handle, ctx)`
- シグネチャ: `function wait_for_server_ready(handle, ctx)`
- 入力: handle, `ctx`
- 出力: なし
- 責務: 起動完了待ち
- 主要ロジック:
  - local は約 20 秒、docker は約 30 秒待つ
  - local プロセス早期終了時はログ出力して失敗
  - Docker では追加診断として `pgrep/ss/od` を実行
- 依存関係: `check_server_alive`, `process_is_running`, `print_server_logs`
- 注意: timeout 時は `StartTimeout`

### `cleanup_existing_server(ctx)`
- シグネチャ: `function cleanup_existing_server(ctx)`
- 入力: `ctx`
- 出力: なし
- 責務: stale プロセス除去
- 主要ロジック:
  - shutdown request
  - listen PID 検索と kill
  - Docker 内/ポート公開中コンテナ停止
- 依存関係: `send_shutdown_request`, `find_listening_pids`, `kill_processes`, `cleanup_docker_processes`
- 注意: 起動前に毎回呼ばれる

### `send_shutdown_request(port)`
- シグネチャ: `function send_shutdown_request(port)`
- 入力: ポート番号
- 出力: なし
- 責務: 穏当な停止要求送信
- 主要ロジック: Java socket で `{"command":"SHUTDOWN"}` を送る
- 依存関係: Java I/O
- 注意: 失敗は握りつぶす

### `pids = find_listening_pids(port)`
- シグネチャ: `function pids = find_listening_pids(port)`
- 入力: ポート番号
- 出力: PID 文字列 cell
- 責務: リスニング PID 特定
- 主要ロジック:
  - Windows: PowerShell → `netstat`
  - Unix: `lsof` → `ss`
- 依存関係: `system`, `split_nonempty_lines`
- 注意: 出力形式差異を正規表現で吸収

### `kill_processes(pids)`
- シグネチャ: `function kill_processes(pids)`
- 入力: PID 群
- 出力: なし
- 責務: プロセス強制終了
- 主要ロジック: Windows は `taskkill /F`、他は `kill -9`
- 依存関係: `system`
- 注意: PID 文字列前後空白を除去

### `cleanup_docker_processes(ctx)`
- シグネチャ: `function cleanup_docker_processes(ctx)`
- 入力: `ctx`
- 出力: なし
- 責務: Docker 内のサーバ残骸除去
- 主要ロジック:
  - compose サービス内 `pkill -f beam_fem_server`
  - 該当公開ポート付きコンテナを `docker stop`
- 依存関係: `detect_docker`, `compose_command`, `shell_quote`
- 注意: Docker 無効なら何もしない

### `handle = start_local_server(binary_path, ctx)`
- シグネチャ: `function handle = start_local_server(binary_path, ctx)`
- 入力: 実行ファイルパス, `ctx`
- 出力: local handle
- 責務: ローカル常駐起動
- 主要ロジック:
  - `temp` ログファイル作成
  - Windows は `Start-Process`、Unix は `nohup ... & echo $!`
  - PID 妥当性を検証
- 依存関係: `make_server_handle`, `ps_quote`, `shell_quote`
- 注意: 標準出力/標準エラーを別ログへ保存

### `handle = start_docker_server(ctx, docker)`
- シグネチャ: `function handle = start_docker_server(ctx, docker)`
- 入力: `ctx`, docker 情報
- 出力: docker handle
- 責務: compose サービス上での常駐起動
- 主要ロジック:
  - `docker compose up -d fsae-sim`
  - `ensure_fem_binary_in_container`
  - `exec -d` で `/workspace/frame_optimize/bin/beam_fem_server 50051` を起動
  - `pgrep` で実際に常駐したか再確認
- 依存関係: `compose_command`, `split_nonempty_lines`
- 注意: detached 起動失敗時は非 detached 診断実行

### `ensure_fem_binary_in_container(docker)`
- シグネチャ: `function ensure_fem_binary_in_container(docker)`
- 入力: docker 情報
- 出力: なし
- 責務: コンテナ内実行ファイルのアーキ整合確保
- 主要ロジック:
  - `uname -m` と ELF `e_machine` を比較
  - 不一致や欠損時はコンテナ内 CMake ビルドを実施
- 依存関係: `compose_command`, `regexp`
- 注意: `od` で ELF ヘッダを読む最小依存設計

### `stop_server(handle)`
- シグネチャ: `function stop_server(handle)`
- 入力: 任意 handle
- 出力: なし
- 責務: サーバ停止
- 主要ロジック:
  - shutdown request を送り、mode に応じ PID kill / docker stop / compose 内 pkill
- 依存関係: `normalize_server_handle`, `cleanup_existing_server`
- 注意: handle 省略時は全掃除

### `alive = check_server_alive()`
- シグネチャ: `function alive = check_server_alive()`
- 入力: なし
- 出力: 真偽
- 責務: 健康確認
- 主要ロジック: `PING` を送り応答文字列有無で判定
- 依存関係: Java socket
- 注意: ポートは固定で `50051`

### `print_server_logs(handle)`
- シグネチャ: `function print_server_logs(handle)`
- 入力: handle
- 出力: なし
- 責務: ログ可視化
- 主要ロジック:
  - local はログ末尾 20 行
  - docker temporary-container は `docker logs`
  - compose-exec は `compose logs --tail 20`
- 依存関係: `print_log_tail`, `detect_docker`
- 注意: handle 空なら何もしない

### `print_log_tail(path_value, label)`
- シグネチャ: `function print_log_tail(path_value, label)`
- 入力: パス, 表示ラベル
- 出力: なし
- 責務: ローカルログ末尾表示
- 主要ロジック: 全文読み込み後、末尾 20 行のみ表示
- 依存関係: `split_nonempty_lines`
- 注意: ログ巨大時でも全文読込なのでコストあり

### `running = process_is_running(handle, ctx)`
- シグネチャ: `function running = process_is_running(handle, ctx)`
- 入力: handle, `ctx`
- 出力: 真偽
- 責務: 起動済み確認
- 主要ロジック:
  - local は `tasklist` または `ps -p`
  - temporary docker container は `docker ps` で確認
- 依存関係: `detect_docker`
- 注意: compose-exec の生存確認は扱わない

### `ctx = get_runtime_context()`
- シグネチャ: `function ctx = get_runtime_context()`
- 入力: なし
- 出力: `project_root`, `compose_file`, `port`, 候補バイナリ一覧など
- 責務: 実行環境情報集約
- 主要ロジック: `frame_root/bin`, `cpp/build*` の候補パス生成
- 依存関係: `get_fem_server_host_port`
- 注意: `ctx.log_dir = frame_optimize/temp`

### `binary_path = find_local_server_binary(ctx)`
- シグネチャ: `function binary_path = find_local_server_binary(ctx)`
- 入力: `ctx`
- 出力: ネイティブ実行可能バイナリパス
- 責務: ローカルバイナリ探索
- 主要ロジック: `is_native_binary` を満たす候補を最初に返す
- 依存関係: `is_native_binary`
- 注意: 非ネイティブ ELF/Mach-O は除外

### `native = is_native_binary(binary_path)`
- シグネチャ: `function native = is_native_binary(binary_path)`
- 入力: バイナリパス
- 出力: 真偽
- 責務: 実行形式判定
- 主要ロジック:
  - Windows は拡張子 `.exe`
  - macOS/Linux は `file` コマンド出力を `Mach-O`/`ELF` 判定
- 依存関係: `system`, `shell_quote`
- 注意: アーキ一致までの厳密判定ではない

### `paths = find_present_candidates(candidates)`
- シグネチャ: `function paths = find_present_candidates(candidates)`
- 入力: パス候補 cell
- 出力: 存在パス cell
- 責務: 診断表示用候補収集
- 主要ロジック: `isfile` 真のものだけ返す
- 依存関係: なし
- 注意: 順序維持

### `docker = try_autostart_docker_desktop(ctx, reason)`
- シグネチャ: `function docker = try_autostart_docker_desktop(ctx, reason)`
- 入力: `ctx`, 失敗理由
- 出力: 再検出後 docker 情報
- 責務: macOS/Windows の Docker Desktop 自動起動
- 主要ロジック:
  - daemon 停止理由のときのみ起動
  - 最大 90 秒待機
- 依存関係: `detect_docker`
- 注意: Linux は対象外

### `docker = detect_docker(ctx)`
- シグネチャ: `function docker = detect_docker(ctx)`
- 入力: `ctx`
- 出力: `available`, `compose_cmd`, `reason`
- 責務: Docker/Compose 利用可否判定
- 主要ロジック:
  - `docker info`
  - `docker compose version` 優先、なければ `docker-compose version`
- 依存関係: `extract_docker_error`, `shell_quote`
- 注意: compose file 不在も明示理由化

### `cmd = compose_command(docker, subcommand)`
- シグネチャ: `function cmd = compose_command(docker, subcommand)`
- 入力: docker struct, subcommand
- 出力: 完全 compose コマンド
- 責務: コマンド連結
- 主要ロジック: `sprintf('%s %s', ...)`
- 依存関係: なし
- 注意: 単純結合

### `handle = make_server_handle(mode, kind, id, display, log_file, error_log)`
- シグネチャ: `function handle = make_server_handle(...)`
- 入力: 各種 handle 属性
- 出力: handle struct
- 責務: handle 作成
- 主要ロジック: struct 化
- 依存関係: なし
- 注意: 他関数で拡張 field を後付与する場合あり

### `handle = normalize_server_handle(handle)`
- シグネチャ: `function handle = normalize_server_handle(handle)`
- 入力: char/string/struct/empty
- 出力: 正規化 handle
- 責務: handle 表現統一
- 主要ロジック:
  - `'external'`, `'exec'`, コンテナ ID 文字列を struct へ変換
  - 欠落 field を補う
- 依存関係: `make_server_handle`
- 注意: `mode` 欠落 struct は無効化

### `desc = describe_server_handle(handle)`
- シグネチャ: `function desc = describe_server_handle(handle)`
- 入力: handle
- 出力: 表示用文字列
- 責務: 人間向け説明生成
- 主要ロジック: `mode/kind` に応じて文言生成
- 依存関係: `normalize_server_handle`
- 注意: `display` 優先

### `text = format_attempt_errors(messages)`
- シグネチャ: `function text = format_attempt_errors(messages)`
- 入力: エラー文配列
- 出力: 整形済み文字列
- 責務: 起動失敗理由の集約
- 主要ロジック: 箇条書き風に連結
- 依存関係: `strjoin`
- 注意: 空配列なら空文字

### `text = join_for_error(values)`
- シグネチャ: `function text = join_for_error(values)`
- 入力: 候補値 cell
- 出力: 連結文字列
- 責務: パス一覧の説明用整形
- 主要ロジック: 空なら `(none)`
- 依存関係: `strjoin`
- 注意: なし

### `msg = extract_docker_error(output)`
- シグネチャ: `function msg = extract_docker_error(output)`
- 入力: `docker info` 出力
- 出力: 要約エラー
- 責務: Docker 失敗理由の人間可読化
- 主要ロジック:
  - daemon 未起動の典型文言を優先判定
  - 次に `ERROR:` 行を採用
  - 最後に末尾 1 行へ縮約
- 依存関係: `split_nonempty_lines`, `trim_single_line`
- 注意: Docker Desktop 起動案内文を返す

### `text = trim_single_line(value)`
- シグネチャ: `function text = trim_single_line(value)`
- 入力: 複数行文字列
- 出力: 末尾有効 1 行
- 責務: エラー文字列縮約
- 主要ロジック: split 後の最後の非空行
- 依存関係: `split_nonempty_lines`
- 注意: 空なら `(no output)`

### `lines = split_nonempty_lines(text)`
- シグネチャ: `function lines = split_nonempty_lines(text)`
- 入力: 文字列
- 出力: 非空行 cell
- 責務: 改行分割
- 主要ロジック: `\r\n|\n|\r` で split
- 依存関係: `regexp`
- 注意: 空行除去

### `container_id = parse_container_id(output)`
- シグネチャ: `function container_id = parse_container_id(output)`
- 入力: docker 出力
- 出力: 12 桁程度の container id
- 責務: ID 抽出
- 主要ロジック: 末尾から 16 進行を探索し 12 文字へ短縮
- 依存関係: `split_nonempty_lines`, `regexp`
- 注意: 本ファイル中では実質未使用

### `quoted = shell_quote(value)`
- シグネチャ: `function quoted = shell_quote(value)`
- 入力: 文字列
- 出力: ダブルクォート付文字列
- 責務: 簡易シェル引用
- 主要ロジック: `"..."` で包む
- 依存関係: なし
- 注意: 厳密エスケープではない

### `quoted = ps_quote(value)`
- シグネチャ: `function quoted = ps_quote(value)`
- 入力: 文字列
- 出力: PowerShell 用引用文字列
- 責務: PowerShell 引数安全化
- 主要ロジック: 単一引用符を二重化して `'...'`
- 依存関係: `strrep`
- 注意: Windows 専用補助
