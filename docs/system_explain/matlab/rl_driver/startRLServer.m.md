# `matlab/rl_driver/startRLServer.m`

## 1. 役割

`startRLServer` は、Docker コンテナ内で C++ 製 `fsae_rl_server` をビルド・再起動し、MATLAB から接続できる状態まで持っていく起動補助関数です。  
単にプロセスを起動するだけでなく、コンテナ起動確認、再ビルド、旧プロセス停止、接続確認までを一括で行います。

## 2. 入口関数

このファイルに定義されているのは `startRLServer(projectRoot)` の 1 関数だけです。

## 3. `startRLServer(projectRoot)` の処理詳細

### 3.1 プロジェクトルート解決

引数が省略された場合、`mfilename('fullpath')` から現在ファイル位置を取得し、`matlab/rl_driver -> sim_overall` と 2 段上がってプロジェクトルートを推定します。

その後 `docker-compose.yml` の存在を確認し、見つからなければ即エラーにします。

### 3.2 接続ポート決定

`hostRlPort = get_rl_server_host_port()` を呼び、ホスト側で MATLAB が接続すべきポート番号を取得します。  
コンテナ内サーバー自体は 12345 番で起動し、Docker のポートマッピングでホスト側へ公開する想定です。

### 3.3 コンテナ起動確認

`docker compose ps --format "{{.State}}" fsae-sim` を実行し、`running` でない場合は `docker compose up -d` でコンテナを立ち上げます。

### 3.4 サーバービルド

コンテナ内 `/workspace/build` で次を実行します。

- `cmake .. -DCMAKE_BUILD_TYPE=Release -Wno-dev`
- `cmake --build . --target fsae_rl_server --parallel`

失敗時はビルド出力を表示して `Failed to build fsae_rl_server` を投げます。

### 3.5 既存サーバー停止

既存の `fsae_rl_server` を止めるため、まず PID ファイルを見て対象 PID を終了します。  
その後、残骸対策として `ps -C fsae_rl_server -o pid=` で列挙した PID も個別に `kill` / `kill -9` します。

コメントどおり、PID ファイル優先で止め、漏れがあればプロセス列挙で補完する方針です。

### 3.6 バックグラウンド起動

`docker compose exec -T ... bash -lc 'nohup /workspace/build/cpp/fsae_rl_server 12345 > /tmp/rl_server.log 2>&1 &'` 形式でコンテナ内常駐起動します。  
`exec -d` では維持できないため、`nohup + &` を採用しています。

同時にサーバー PID を PID ファイルへ保存します。

### 3.7 起動後の接続確認

最大 15 回、1 秒間隔で以下を試します。

1. `FSAESimClient('localhost', hostRlPort)` を生成
2. `connect()`
3. `init('circle')`
4. 成功したら `disconnect()` して ready 判定

ready にならない場合はコンテナ内ログ末尾を表示し、エラー終了します。

## 4. 実装上の要点

- MATLAB からの利用者は本関数 1 つで「コンテナ確認 → ビルド → 再起動 → 疎通確認」まで完結できます。  
- `FSAESimClient` を使った実接続テストまで含むため、単なるプロセス起動より信頼性が高いです。  
- サーバーの listen ポートはコンテナ内 12345、MATLAB の接続先は `get_rl_server_host_port()` が返すホスト側ポートです。

## 5. 注意点

- PID ファイルとログ出力先に `/tmp/rl_server.pid` / `/tmp/rl_server.log` を使っています。  
- Docker Compose、`fsae-sim` サービス名、コンテナ内 `/workspace/build` 構成に強く依存します。  
- `init('circle')` が readiness probe なので、`circle` トラックがサーバー側で利用可能であることが前提です。
