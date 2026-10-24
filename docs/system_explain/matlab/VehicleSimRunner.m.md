# `matlab/VehicleSimRunner.m`

## 1. ファイルの目的

Docker コンテナ内の `cpp/laptime_sim` を MATLAB から継続利用しやすくするラッパークラスです。  
都度 `docker compose run` を起こさず、既存コンテナへ `exec` して JSON パラメータを渡し、結果 JSON を MATLAB 構造体へ戻します。

## 2. 主な入出力または依存関係

- 主入力:
  - `project_root`
  - `container_name`
  - `params_struct`
  - `track_name`, `track_radius`, `sim_time`
  - オプション `driver_mode`, `rl_actor_dir`
- 主出力:
  - `result`（C++ 標準出力から抽出した JSON）
  - `success`
- 依存関係:
  - Docker / docker compose
  - `output/runtime/` への一時 JSON 書き込み
  - RL 利用時は `matlab/rl_driver/rl_actor_export` か指定ディレクトリ内のエクスポート済みファイル

## 3. 処理の流れ / アルゴリズム

1. コンストラクタが `docker-compose.yml` の場所を推定し、`docker compose exec -T` コマンドを組み立てます。
2. `ensureContainerRunning()` でコンテナの起動状態を確認し、未起動なら `up -d` します。
3. `run()` は入力車両パラメータを `output/runtime/params_<timestamp>.json` に保存します。
4. トラック条件と driver 引数を付けて `./cpp/laptime_sim --output_json --params_file ...` をコンテナ内で実行します。
5. 終了後に一時 JSON を削除し、標準出力中の `{...}` 部分だけを切り出して `jsondecode()` します。
6. RL ドライバ指定時は `buildDriverArgs()` がプロジェクト配下にあることと必要ファイルの存在を検証します。

## 4. クラスと各メソッド

### `classdef VehicleSimRunner < handle`

- `handle` 継承なので、同一インスタンス内で `ProjectRoot` や `DockerCmd` を共有し続けます。

### プロパティ

- `RawContainerName`: docker compose service 名
- `ProjectRoot`: `docker-compose.yml` を含む想定ルート
- `DockerCmd`: 実際に使う `docker compose ... exec -T ...` コマンド文字列

### `VehicleSimRunner(project_root, container_name)`

- ルートディレクトリとコンテナ名を初期化します。
- 指定ルート直下に `docker-compose.yml` がなければ親ディレクトリも確認します。
- `-T` 付き `exec` を使うことで MATLAB の `system()` 呼び出しで TTY ハングしないようにしています。
- 最後に `ensureContainerRunning()` を呼び、インスタンス生成時点で実行可能状態にします。

### `run(obj, params_struct, track_name, track_radius, sim_time, varargin)`

- 主実行メソッドです。
- 入力パラメータを JSON 化してホスト側 `output/runtime/` に保存し、コンテナ内 `/workspace/output/runtime/...` として参照させます。
- `driver_mode` は `pid` / `rl` を受け付け、`buildDriverArgs()` に処理を委譲します。
- シミュレーション失敗時は `result=[]`, `success=false` を返します。
- 標準出力から JSON を抽出するため、前後に混在するログ行を許容します。
- `DEBUG:` 行があれば最後に MATLAB 側へ表示します。

### `buildDriverArgs(obj, driver_mode, rl_actor_dir)`

- `pid` なら `--driver pid` を返します。
- `rl` なら RL actor directory を確定し、プロジェクト外参照を禁止した上で必要 3 ファイルの存在を確認します。
- その後、ホストパスを `/workspace/...` 形式へ変換して `--driver rl --rl-actor-dir ...` を返します。

### `ensureContainerRunning(obj)`

- `docker compose exec ... echo ok` でコンテナ到達性を確認します。
- 失敗時は `docker compose up -d` を実行し、最大 30 秒待機して再確認します。
- 起動できない場合は warning を出し、強制終了まではしません。

## 5. 注意点・前提条件

- 一時 JSON は `run()` のたびに作成・削除されるため、`output/runtime/` への書込権限が必要です。
- JSON 抽出は最初の `{` と最後の `}` を使う単純方式なので、標準出力に別の JSON 断片が混じると誤解析の可能性があります。
- RL actor directory は必ずプロジェクト配下である必要があります。
- `run()` はラップタイムシミュレータの標準出力契約に依存しているため、C++ 側 CLI 仕様変更の影響を受けます。
