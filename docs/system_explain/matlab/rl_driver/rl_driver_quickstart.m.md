# `matlab/rl_driver/rl_driver_quickstart.m`

## 役割

RL ドライバー運用の入口となるクイックスタート用スクリプトです。  
自分で学習ロジックを持つのではなく、**環境準備・接続確認・学習起動・評価・受け入れ判定・C++ エクスポート・復旧運転**を section 実行で辿れるよう整理しています。

## このファイルの性格

- MATLAB エディタで `%%` ごとに実行する前提です。
- 各 section は「何を最初に試すべきか」を示す運用レシピです。
- 実処理本体は `startRLServer`、`trainCircleSimple`、`trainRLDriver`、`trainWithCurriculum`、`validateRLReadiness` などの別関数に委譲します。

## 実行フロー

1. パスを通して RL 関連関数を呼べる状態にする。
2. Docker 上の C++ RL サーバーを起動する。
3. `FSAESimClient` で TCP 接続と `step` 応答を確認する。
4. 目的に応じて単純学習・円専用学習・カリキュラム学習・recovery 再開・ace-driver 継続を選ぶ。
5. 学習済みエージェントを評価し、受け入れ基準を満たすか判定する。
6. 学習済み actor を C++ 用にエクスポートする。
7. サーバープロセスの確認・停止・再起動を行う。

## section 詳細

### `%% RL_DRIVER_QUICKSTART.m`

冒頭コメント部です。  
このファイルが「全コマンドの目次」であり、MATLAB の Section 実行で使うことを明示しています。

### `%% ========== 0. 初期設定（毎回まず実行）==========`

```matlab
RL_DIR = fileparts(mfilename('fullpath'));
addpath(RL_DIR);
addpath(fileparts(RL_DIR));
```

- このファイル自身のディレクトリを `RL_DIR` に取得します。
- `matlab/rl_driver` とその親 `matlab` を MATLAB パスへ追加します。
- 以後、同ディレクトリ内の学習関数や補助関数をカレント位置に依存せず呼べるようにします。
- 最初に必ず実行する section です。

### `%% ========== 1. Dockerビルド＋サーバー起動 ==========`

```matlab
startRLServer();
```

- Docker コンテナ側で C++ シミュレータ／RL サーバーを起動します。
- MATLAB 側の学習環境 `FSAEEnvironment` や `FSAESimClient` はこのサーバーとの通信を前提にしているため、学習や評価の前段に必要です。
- コメント上では既定ホストポート `22345` を使う運用が想定されています。

### `%% ========== 2. 接続テスト ==========`

```matlab
client = FSAESimClient('localhost', get_rl_server_host_port());
client.connect();
trackLen = client.init('circle');
obs = client.reset();
[obs2, info] = client.step(0.0, 0.5, 0.0);
trackLen2 = client.init('autocross');
client.disconnect();
```

- `FSAESimClient` を直接使い、MATLAB から TCP 接続できるかを確認します。
- `get_rl_server_host_port()` によりポート番号を取得し、設定ハードコードを避けています。
- `init('circle')` で circle コースを初期化し、コース長が返ることを確認します。
- `reset()` で初期観測を受け取り、`step(0.0, 0.5, 0.0)` でステア 0、加速 0.5、ブレーキ 0 の一歩進行を試します。
- `info(1)` と `info(4)` を表示し、少なくとも速度と進捗が返ることを確認します。
- 続けて `autocross` の初期化も行い、複数トラックが使えることを確認します。
- 最後に `disconnect()` でセッションを閉じます。

### `%% ========== 3a. 円形コース特化学習（推奨: まずこれから）==========`

```matlab
trainCircleSimple();
```

- 最も導入しやすい円形コース専用学習を呼びます。
- 2D 行動空間、単一タスク、保守的な報酬設計で、まず完走挙動を作るための入口です。

### `%% ========== 3b. 単純学習（1つのトラック）==========`

```matlab
trainRLDriver('circle');
```

- 汎用学習関数 `trainRLDriver` を最小引数で使う例です。
- 指定トラック 1 つに対して SAC エージェントを学習させます。
- quickstart では `circle` を例示していますが、`autocross` や `skidpad` に切り替えることもできます。

### `%% ========== 3c. カリキュラム学習 ==========`

```matlab
trainWithCurriculum('autocross', 'RandInterval', 100);
```

- 直線 → 円 → スキッドパッド → オートクロスという段階学習の呼び出し例です。
- `RandInterval=100` により、ドメインランダム化対象の車両条件を 100 エピソードごとに更新する運用を示します。
- 最終目標トラックは `autocross` です。

### `%% ========== 3d. ランダム化なしでカリキュラム学習 ==========`

```matlab
trainWithCurriculum('autocross', 'DomainRand', false, 'RandInterval', 100);
```

- ドメインランダム化を切って、まず制御則だけを安定させたい場合の例です。
- `RandInterval` は渡されていますが、`DomainRand=false` のため実質的には無効化前提の補助引数です。

### `%% ========== 3e. ランダム化の強度を調整 ==========`

```matlab
trainWithCurriculum('autocross', 'RandStrength', 0.5, 'RandInterval', 100);
```

- ランダム化の幅だけを強めたり弱めたりする例です。
- コメントにある通り、`RandStrength` は 0.0〜1.0 の規模指定です。
- 固定パラメータ学習と高ばらつき学習の中間設定を試す section です。

### `%% ========== 3f. 特定パラメータで学習 ==========`

```matlab
artifact = load_vehicle_artifact('../../optimized_autocross.json');
trainRLDriver('autocross', artifact.params);
```

- JSON artifact から車両パラメータを読み込み、その固定条件で学習する例です。
- `artifact.params` を `FSAEEnvironment` の車両パラメータとして渡すため、最適化済み車両や設計候補を学習条件へ反映できます。

### `%% ========== 3g. 端末停止後の phase 3/4 recovery 再開 ==========`

```matlab
resumePhase34RecoveryAfterRestart();
```

- long run 中に端末や MATLAB セッションが落ちた後の再開導線です。
- コメントの通り、既存 deployment の温存、fallback の公開、checkpoint / progress に基づく再開を自動化する想定です。
- 運用上の復旧 section であり、学習アルゴリズム本体ではなくジョブ継続性の確保が目的です。

### `%% ========== 3h. recovery 完了後の ace-driver 継続学習 ==========`

```matlab
trainAceDriverFromRecovery();
```

- 直近の recovery `final_agent.mat` を再開元にして、より高速な autocross 指向へ継続学習します。
- review トラックを残した mixed-track continuation なので、速さを上げつつ circle/skidpad 系の能力も維持しやすい構成です。

### `%% ========== 3i. 端末停止後の ace-driver 継続再開 ==========`

```matlab
resumeAceDriverAfterRestart();
```

- ace run 側の `progress.json` / `AgentXX.mat` を優先して安全に再開します。
- 既存 deployment を守りながら、高速化 run だけを継続するための運用 section です。

### `%% ========== 4. 学習済みエージェントで評価 ==========`

```matlab
evaluateRLDriver('rl_results/curriculum/final_agent.mat', 'autocross');
```

- 保存済み `.mat` エージェントを指定トラックで評価します。
- quickstart では curriculum 学習結果を `autocross` で確認する例になっています。
- 学習完了直後の sanity check として使う section です。

### `%% ========== 4b. 受け入れ基準チェック（3トラック）==========`

```matlab
validateRLReadiness(...);
```

- `autocross`、`skidpad`、`circle` の 3 トラックで readiness を評価します。
- コメント上の評価軸は完走率・ラップタイム・コース逸脱率です。
- `Episodes=5` なので、各トラックを少数反復して簡易的な合否をまとめます。
- `SaveJson` に JSON レポートを出力し、後段の CI やレビューで再利用しやすくしています。

### `%% ========== 4c. 受け入れゲート一括実行（推奨）==========`

```matlab
gate = runRLReadinessGate(...);
disp(gate.summary.overall_pass);
disp(gate.report_path);
```

- エクスポート、MATLAB↔C++ 一致性確認、受け入れ判定をまとめて実行する上位 section です。
- `ActorExportDir` に C++ 推論用 actor を出力します。
- `gate.summary.overall_pass` と `gate.report_path` を表示し、最終合否とレポート位置をすぐ確認できるようにしています。

### `%% ========== 4d. 最新 recovery final_agent の受け入れゲート ==========`

- `phase34_recovery_from_phase3_final/final_agent.mat` を明示指定して readiness gate を回す例です。
- 今回完走した recovery run の成果物を、その保存先ディレクトリへレポートを残しながら評価する用途を想定しています。

### `%% ========== 5. C++用エクスポート（学習後）==========`

```matlab
agentData = load('rl_results/curriculum/final_agent.mat');
deployVehicleParams = resolve_rl_vehicle_params();
exportActorForCpp(agentData.agent, 'rl_actor_export', deployVehicleParams);
```

- 学習済みエージェントを C++ ランタイム用に変換する section です。
- `resolve_rl_vehicle_params()` で deployment 時に使う車両パラメータを解決します。
- コメントにある通り、非デフォルト車両を使うならここで上書きするのが意図です。

### `%% ========== 6. サーバー管理コマンド ==========`

```matlab
PROJECT_ROOT = fileparts(fileparts(RL_DIR));
COMPOSE = sprintf('docker compose -f "%s/docker-compose.yml"', PROJECT_ROOT);
EXEC    = sprintf('%s exec -T fsae-sim bash -c', COMPOSE);
```

- RL サーバーが動く `fsae-sim` コンテナへコマンドを送るための文字列を組み立てます。
- `PROJECT_ROOT` は `matlab/rl_driver` の 2 階層上、すなわちリポジトリ側の想定ルートです。

続く 3 つの処理は次の通りです。

1. **状態確認**
   - `ps aux | grep fsae_rl_server | grep -v grep` を実行し、サーバープロセスの有無を表示します。
2. **停止**
   - `pkill -f fsae_rl_server; true` をコンテナ内で実行し、停止済みでもエラー終了しないようにしています。
3. **再起動**
   - 再度 `startRLServer()` を呼んで、既定の起動手順へ戻します。

## このファイルが生成・参照する主な成果物

- 学習済み agent: `rl_results/.../*.mat`
- readiness レポート: `rl_results/.../readiness_report.json`
- C++ 用 actor 出力: `rl_actor_export`
- Docker 上の RL サーバープロセス

## 注意点

- section 間で前提状態があるため、通常は **0 → 1 → 2 → 3/4/5/6** の順で使います。
- このファイルはガイドなので、失敗時の詳細制御は委譲先関数側で行います。
- `pkill` を使う停止処理は quick operation 向けで、厳密な PID 管理ではありません。
