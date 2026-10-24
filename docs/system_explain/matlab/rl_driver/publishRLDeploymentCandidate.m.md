# `matlab/rl_driver/publishRLDeploymentCandidate.m`

## 役割

`publishRLDeploymentCandidate` は、選んだ RL Agent を **安定 deployment 用パスへ昇格公開する関数**です。  
単に `deployment_agent.mat` を保存するだけでなく、C++ 実行系が読む Actor export、公開メタデータ、車両パラメータ情報まで揃えて一式を更新します。

---

## deployment flow の全体像

1. 入力が Agent オブジェクトか path かを判定する。
2. path の場合は `loadRLAgentFromFile` で Agent を読む。
3. `assertAgentObservationDimension` で現行観測契約と一致するか確認する。
4. `exportActorForCpp` を呼び、deployment 用 Actor export を更新する。
5. `deployment_agent.mat` を保存する。
6. 公開メタデータを JSON として deployment directory と actor export directory の両方へ書く。
7. deployment 先 path 群を返す。

このため publish は「MATLAB 保存」と「C++ 側参照ファイル更新」を同時に完了させる操作になっています。

---

## 既定ディレクトリ

### `ActorExportDir`

未指定時は `matlab/rl_driver/rl_actor_export` を使います。  
ここは MATLAB 側ソースの近くにある既定 export 置き場です。

### `DeploymentDir`

未指定時は `<projectRoot>/rl_results/deployment` を使います。  
ここに `deployment_agent.mat` と `deployment_metadata.json` が作られます。

---

## 関数詳細

## `publishRLDeploymentCandidate(agentOrPath, varargin)`

### 入力

| 引数 | 意味 |
| --- | --- |
| `agentOrPath` | Agent オブジェクト、または `.mat` path |
| `ActorExportDir` | C++ 用 Actor export 先 |
| `DeploymentDir` | deployment 管理ディレクトリ |
| `VehicleParams` | export 時に使う車両パラメータ override |
| `Metadata` | 公開理由や発行元などを追加する任意 struct |

### metadata の基本項目

関数はまず以下を自動生成します。

- `generated_at`
- `source_agent_path`
- `source_variable`
- `deployment_agent_path`
- `actor_export_dir`
- `vehicle_params`

その後、`Metadata` が与えられていれば `merge_nested_structs` で再帰マージします。  
そのため既定 metadata を保ったまま、`published_reason` や `source_run_dir` を追加できます。

### 保存物

- `<DeploymentDir>/deployment_agent.mat`
- `<DeploymentDir>/deployment_metadata.json`
- `<ActorExportDir>/deployment_metadata.json`
- `<ActorExportDir>/rl_actor_weights.txt`
- `<ActorExportDir>/rl_normalization_params.txt`
- `<ActorExportDir>/rl_actor_test_data.csv`

### 返り値

- `deployment_agent_path`
- `actor_export_dir`
- `metadata_path`

---

## `resolveDeploymentAgent(agentOrPath)`

publish 入力を統一する helper です。

- 文字列 / string なら path とみなし、`loadRLAgentFromFile` で Agent を読む
- それ以外は既に Agent オブジェクトとみなしてそのまま使う

path 指定時は `sourcePath` と `sourceVarName` も返すため、metadata に元ファイルの出所を残せます。  
一方、Agent オブジェクト直渡しではこれらは空文字のままです。

---

## `writePrettyJson(filePath, value)`

`jsonencode(..., 'PrettyPrint', true)` で整形 JSON を保存する helper です。  
親ディレクトリが無ければ `mkdir` し、ファイルを開いてそのまま書き込みます。

この helper により、deployment metadata は人間が読みやすい整形 JSON で残ります。

---

## deployment 公開の運用上の意味

- `trainWithCurriculum` の通常完了では run-local export のみで止める
- readiness を通した候補だけ `publishRLDeploymentCandidate` で既定 deployment へ昇格する
- `resumePhase34RecoveryAfterRestart` では recovery 開始前の safe fallback 公開にも使う

つまりこの関数は、学習成果物を「単なる出力」から「運用系が参照する候補」へ格上げする境界です。
