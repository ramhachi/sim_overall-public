# `matlab/rl_driver/runRLReadinessGate.m`

## 1. 役割

`runRLReadinessGate` は、RL ドライバーを実運用候補として扱う前に必要な処理を **一気通貫で実行する上位ゲート** です。  
単なる評価関数ではなく、MATLAB 側 agent を C++ 系実行環境へ橋渡しし、その整合性まで含めて確認します。

コメントにある通り、処理は大きく 3 段です。

1. MATLAB actor を export
2. C++ 側 `rl_actor_check` で整合確認
3. `validateRLReadiness` で多トラック readiness 判定

さらに条件を満たせば deployment 公開まで行えます。

---

## 2. 入口関数 `result = runRLReadinessGate(agentPath, varargin)`

### 2.1 入力

- `agentPath`
  - 既定では `rl_results/curriculum/final_agent.mat`
  - readiness 対象 agent の保存ファイル
- `varargin`
  - ゲートの振る舞いを変えるオプション

### 2.2 主なオプション

- `ActorExportDir`
  - 公開用 actor export の基準ディレクトリ
- `Tracks`
  - readiness 判定に使うトラック一覧
- `Episodes`
  - 各トラック評価回数
- `Threshold`
  - `rl_actor_check` の一致許容値
- `SaveDir`
  - readiness レポートや staging export を置く場所
- `StartServer`
  - true ならこの関数内で RL サーバを起動
- `VehicleParams`
  - export/評価に使う車両パラメータ
- `DomainRandStrength`, `DomainRandInterval`
  - readiness 評価時の domain randomization 設定
- `MinCompletionRate`, `MaxOffCourseRate`, `MaxLapTimeRatio`
  - readiness 合格基準
- `PIDBaselineLapTime`
  - PID 基準タイム struct
- `PublishIfPass`
  - 合格時に deployment 公開まで進めるか
- `DeploymentDir`
  - deployment 出力先
- `Metadata`
  - deployment 公開時に追加するメタデータ

### 2.3 出力 `result`

戻り値には主に次が入ります。

- `actor_export_dir`
  - staging export の絶対パス
- `publish_actor_export_dir`
  - 公開先 export ディレクトリの絶対パス
- `report_path`
  - readiness JSON レポート
- `summary`
  - `validateRLReadiness` の返り値
- `deployment_published`
  - 公開したかどうか
- `deployment`
  - 公開時の結果 struct

---

## 3. 主処理の流れ

### 3.1 オプション解析と agent 読み込み

`inputParser` で全設定を受け取り、`loadRLAgentFromFile` で agent をロードします。  
続いて `assertAgentObservationDimension` で観測次元を検証します。

この時点で、後続の export・検証・評価がすべて同じ agent を起点に進みます。

### 3.2 プロジェクト基準パスの導出

`mfilename('fullpath')` から

- `thisDir` = `matlab/rl_driver`
- `matlabDir` = `matlab`
- `projectRoot` = リポジトリ root

を導きます。  
以後のパス解決は、この 3 つを基準に `resolvePath` へ委譲されます。

### 3.3 保存先ディレクトリの決定

以下を `resolvePath` で解決します。

- `actorExportDir`
- `saveDir`
- `deploymentDir`

`saveDir` が存在しなければ `mkdir` します。  
ここは readiness report や staging export を置く中間成果物ディレクトリです。

### 3.4 staging export の作成

公開先を直接上書きせず、まず

`actor_export_candidate_<timestamp>`

という時刻付き staging ディレクトリを `saveDir` 配下に作ります。  
そこへ `exportActorForCpp(agent, stagingActorExportDir, opts.VehicleParams)` を実行します。

この設計により、未検証 actor がいきなり正式 export を汚さないようにしています。

### 3.5 RL サーバ起動制御

`opts.StartServer` が true の場合は `startRLServer(projectRoot)` を呼びます。  
ただし readiness validation 呼び出し時には `StartServer` を false に切り替えています。

これは同じゲート内でサーバ起動要求を二重に出さないための調整です。

### 3.6 C++ 整合確認 `rl_actor_check`

staging export の絶対パスを canonical 化し、さらに

- staging export が project root 配下にあること
- Docker コンテナから見える `/workspace/...` 相対パスへ変換できること

を確認します。

その後、`docker compose ... exec -T fsae-sim ... ./cpp/rl_actor_check` を `system` で実行します。  
使う引数は次です。

- `--actor-dir`
- `--test-data`
- `--threshold`

`checkStatus ~= 0` なら即 error で停止します。  
つまり、**C++ 側推論一致が取れない agent は readiness 評価へ進めない** 仕様です。

### 3.7 readiness 検証

`validateRLReadiness` を呼び、以下を引き渡します。

- トラック一覧
- エピソード数
- 車両パラメータ
- domain randomization 設定
- 完走率 / 逸脱率 / ラップ比の閾値
- PID 基準タイム
- JSON 保存先

ここで得た `summary` が、配備候補としての最終判断材料になります。

### 3.8 公開可否判定

自動公開は次の条件をすべて満たす場合のみです。

1. `summary.overall_pass`
2. `opts.PublishIfPass`
3. `hasPidBaselineForTracks(opts.PIDBaselineLapTime, opts.Tracks)` が true

PID 基準が不足していると、たとえ `overall_pass` でも自動公開はスキップされます。  
これは readiness の比較条件が不完全なまま deployment を進めないための安全策です。

### 3.9 deployment 公開

公開条件を満たした場合は、`readiness_report_path` と `readiness_summary` を `Metadata` にマージし、`publishRLDeploymentCandidate` を呼びます。  
結果は `result.deployment` に格納され、`deployment_published = true` になります。

---

## 4. ローカルヘルパー `resolvePath(pathIn, rlDir, matlabDir, projectRoot)`

### 4.1 役割

相対/絶対が混在する入力パスを、RL ディレクトリ・`matlab` ディレクトリ・project root のいずれか基準で解決する helper です。

### 4.2 判定順

1. `isAbsolutePath(pathIn)` が true ならそのまま返す
2. `isfolder(pathIn)` または `isfile(pathIn)` ならそのまま返す
3. 以下 3 候補を作る
   - `c1 = fullfile(rlDir, pathIn)`
   - `c2 = fullfile(matlabDir, pathIn)`
   - `c3 = fullfile(projectRoot, pathIn)`
4. 親ディレクトリの存在状況を見て `c1` / `c2` / `c3` のどれかを返す

### 4.3 意味

呼び出し側が `rl_driver` 相対で考えても、`matlab` 相対で考えても、project root 相対で考えても動きやすくするための吸収層です。

---

## 5. ローカルヘルパー `isAbsolutePath(pathIn)`

### 5.1 役割

入力パスが絶対パスかどうかを判定します。

### 5.2 判定ロジック

- Unix 系: `startsWith(pathStr, filesep)`
- Windows 系: 正規表現 `^[A-Za-z]:[\\/]`

クロスプラットフォームな最低限の絶対パス判定 helper です。

---

## 6. ローカルヘルパー `hasPidBaselineForTracks(pidBaselineLapTime, tracks)`

### 6.1 役割

指定された全トラックについて、有効な PID 基準ラップタイムが struct 内に揃っているかを確認します。

### 6.2 チェック内容

各トラックについて

1. `matlab.lang.makeValidName` でフィールド名化
2. そのフィールドが存在するか
3. 値が
   - 空でない
   - 数値
   - スカラー
   - `> 0`

を満たすかを順に確認します。

1 つでも欠ければ false を返し、即終了します。

### 6.3 なぜ必要か

自動公開条件に readiness の比較基準不足を持ち込まないためです。  
「走れた」だけではなく、「既知の PID 基準に対し妥当な速さか」まで確認できる状態でのみ publish する意図があります。

---

## 7. このファイルのシステム上の位置付け

本ファイルは RL 配備前パイプラインの **最上位ゲート** です。

- `evaluateRLDriver`  
  目視評価
- `validateRLReadiness`  
  定量評価
- `runRLReadinessGate`  
  export、C++ 整合、定量評価、必要なら公開までを統合

したがって「学習済み agent を production 候補に昇格させてよいか」を最も広い視点で判定するのがこのファイルです。

---

## 8. 注意点

- C++ 整合確認は Docker コンテナ `fsae-sim` と `/workspace` マウント前提です。
- staging export は project root 配下であることが必須で、外部パスは明示的に reject されます。
- readiness 合格だけでは自動 publish されず、PID 基準の完全性も要求されます。
