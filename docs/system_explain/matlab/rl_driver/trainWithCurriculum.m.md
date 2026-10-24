# `matlab/rl_driver/trainWithCurriculum.m`

## 役割

`trainWithCurriculum` は、このリポジトリの MATLAB 側 RL 学習を実際に回す中核関数です。  
直線 → 円旋回 → スキッドパッド → オートクロスという 4 段階カリキュラムを構築し、各フェーズで環境を作成し、必要なら Domain Randomization を有効化し、学習結果・進捗・最終エクスポートをまとめて管理します。

通常学習だけでなく、`trainPhase34Recovery.m` から recovery 用プリセットを受け取る実行本体でもあります。

---

## 処理の全体像

1. `inputParser` で学習条件を受け取る。
2. 既定のトラックプール・報酬重みを作る。
3. `PhaseTrackPools` / `PhaseRewardOverrides` があれば既定値へ上書きする。
4. 4 フェーズの `curriculum` セル配列を組み立てる。
5. `ResumeAgent` と `progress.json` を見て開始フェーズを決める。
6. 各フェーズで `FSAEEnvironment` を構築し、報酬重み・環境上書き・DR を適用する。
7. 新規 Agent 作成または再開 Agent 検証後、`train(agent, env, trainOpts)` を実行する。
8. `agent.mat` / `training_stats.mat` / `progress.json` を保存する。
9. 全フェーズ終了後に `final_agent.mat` を保存し、`exportActorForCpp` で C++ 向け Actor を吐き出す。
10. `PublishFinalToDefault=true` のときだけ `publishRLDeploymentCandidate` で既定 deployment へ公開する。

---

## 主要オプション

| 引数 | 意味 |
| --- | --- |
| `DomainRand` | 環境 reset 時の車両パラメータランダム化を有効化するか |
| `RandInterval`, `RandStrength` | DR の切替周期と強度 |
| `ResumeAgent` | 既存 Agent から再開するときの Agent オブジェクト |
| `SaveBaseDir` | 各フェーズ成果物と `progress.json` を保存する run-local ディレクトリ |
| `PhaseEpisodes`, `PhaseSteps` | 各フェーズの最大エピソード数・ステップ数 |
| `EnvOverrides` | `FSAEEnvironment` のプロパティ上書き |
| `PhaseTrackPools` | フェーズごとの候補トラック群 |
| `PhaseRewardOverrides` | フェーズごとの報酬重み上書き |
| `StartPhase` | 明示的に再開フェーズを固定する override |
| `CompletedEpisodesOffset` | 再開時に環境へ渡す総エピソードオフセット |
| `SaveAgentCriteria`, `SaveAgentValue` | `AgentXX.mat` チェックポイント生成条件 |
| `FinalActorExportDir` | 最終 Actor エクスポート先 |
| `PublishFinalToDefault` | 学習完了後に deployment 公開まで行うか |
| `PublishVehicleParams` | export / publish 時に使う車両パラメータ上書き |

---

## フェーズ構成とカリキュラム

既定では以下の順で進みます。

1. `straight`
2. `circle`
3. `skidpad`
4. `autocross`

ただし実装上は「代表トラック名」と「実際に sampling するトラックプール」を分けています。  
たとえば phase 4 は `autocross` を代表名にしつつ、`skidpad` や `circle_*` を混ぜた review 学習へ拡張できます。

`finalTrackName` が `autocross` 以外なら phase 4 の代表トラックを差し替えます。さらに代表名が重複した場合は `unique(...,'stable')` により重複フェーズを削除します。  
そのため「最終フェーズだけ差し替える」だけでなく、「結果として 3 フェーズ構成になる」ケースもあります。

---

## 再開とチェックポイント管理

### `progress.json`

`SaveBaseDir/progress.json` には最低限以下を記録します。

- `last_completed_phase`
- `track`
- `completed_episodes`
- `timestamp`

`ResumeAgent` が与えられ、かつ `StartPhase` 未指定で `progress.json` が存在すると、関数はこのファイルから次の開始フェーズを決めます。  
つまり再開時の主判定は「外から phase を渡す」ではなく「run-local progress を読む」が基本です。

### `AgentXX.mat`

`trainOpts.SaveAgentDirectory = saveDir` と `SaveAgentCriteria` / `SaveAgentValue` により、MATLAB RL Toolbox 標準の保存規則で途中チェックポイントが生成されます。  
これらは recovery 系関数が途中再開位置を決める材料にもなります。

### `CompletedEpisodesOffset`

再開時は `env.EpisodeIndexOffset = completedEpisodes` を設定します。  
これにより学習エピソード番号とログ番号を run 全体で連続させられます。

---

## 最終エクスポートと deployment 公開

全フェーズ終了後、関数は必ず `final_agent.mat` を保存します。  
その後、既定では `final_actor_export/` へ `exportActorForCpp` を実行し、C++ 側が読むテキスト形式へ変換します。

重要なのは、**学習完了 = 即 deployment 切替ではない** 点です。  
コメントにもある通り、既定 path への公開は readiness を通した候補だけに限定する方針で、通常は run-local export のみを行います。  
`PublishFinalToDefault` を true にしたときだけ `publishRLDeploymentCandidate` が呼ばれます。

---

## 関数詳細

## `trainWithCurriculum(finalTrackName, varargin)`

### 役割

4 フェーズカリキュラム学習の本体です。

### 重要な内部処理

- `PhaseEpisodes` / `PhaseSteps` は必ず 4 要素でなければならず、違えば即エラーです。
- `getDefaultCurriculumTrackPools()` と `getDefaultCurriculumRewardOverrides()` で既定設定を作り、必要なら override を重ねます。
- `getVehicleParamBounds()` の返り値を DR 用境界として環境へ渡します。
- `ResumeAgent` が無い最初のフェーズだけ `createSACAgent(env)` を呼びます。
- 再開 Agent がある場合は `assertAgentObservationDimension` で観測次元の整合性を強制します。
- フェーズごとに `saveDir = phase%d_<track>` を作り、`agent.mat` と `training_stats.mat` を保存します。
- フェーズ完了ごとに `completedEpisodes` を加算し、`progress.json` を更新します。
- 各フェーズ後に `delete(env)` して 1-client 前提の RL server 接続を確実に解放します。

### 出力成果物

- `<SaveBaseDir>/phaseN_<track>/agent.mat`
- `<SaveBaseDir>/phaseN_<track>/training_stats.mat`
- `<SaveBaseDir>/phaseN_<track>/AgentXX.mat`（条件一致時）
- `<SaveBaseDir>/progress.json`
- `<SaveBaseDir>/final_agent.mat`
- `<FinalActorExportDir or final_actor_export>/*`

---

## `getDefaultCurriculumTrackPools()`

4 フェーズ分の既定トラックプールを返します。

- phase 1: `straight`, `acceleration`
- phase 2: `circle` 系
- phase 3: `skidpad` を厚めにしつつ `figure_eight`, `circle` 系を混ぜる
- phase 4: `autocross` を主軸に review 用トラックを混ぜる

`trainPhase34Recovery` はこの既定値を丸ごと差し替えるのではなく、一部フェーズだけ recovery 向けに上書きして使います。

---

## `getDefaultCurriculumRewardOverrides(phaseSteps)`

フェーズごとの既定報酬重み struct を返します。  
実装上、引数 `phaseSteps` は現在使われておらず、戻り値は固定です。

主な意図は以下です。

- phase が進むほど `W_boundary`, `W_centering`, `W_heading` を強くする
- 最終 phase では `W_alive` を追加して継続走行をわずかに押す

---

## `resolvePhaseTrackPools(defaultTrackPools, overrideTrackPools)`

トラックプールの override 解決器です。

- override 未指定なら既定値をそのまま返す
- 4 フェーズ分のセル配列でなければエラー
- 各要素は `normalizeTrackPool` で文字列セルへ正規化
- 空要素は「その phase は既定値維持」の意味

recovery 学習ではこの関数経由で phase 3, 4 の mixed-track 構成を注入します。

---

## `resolvePhaseRewardOverrides(defaultRewardOverrides, overrideRewardOverrides)`

報酬重み override を解決します。

- override 未指定なら既定値を返す
- 各 phase 要素は struct 必須
- `merge_nested_structs` で既定値へ再帰マージするため、部分上書きが可能

つまり recovery 側は `W_boundary` だけ変え、他の重みは既定を維持する、といった使い方ができます。

---

## `normalizeTrackPool(value, phaseIndex)`

トラックプール入力を最終的に `cellstr` へ揃える補助関数です。

受け入れる形は以下です。

- 文字列スカラー
- string scalar
- string 配列
- 文字列セル配列

不正な要素型や空配列は phase 番号付きでエラーにします。  
入力不正を早い段階で潰し、学習中に曖昧な track 名が混入しないようにしています。

---

## `readProgressFile(progressFile)`

`progress.json` を `jsondecode` で読み、最低限必要なフィールドがあるか検査します。  
必須なのは `last_completed_phase` と `track` です。片方でも欠けると silent fallback せずエラーにします。

この厳格さにより、壊れた progress に気づかず誤ったフェーズから再開する事故を防ぎます。

---

## `writeProgressFile(progressFile, progData)`

進捗 JSON を安全寄りに更新する helper です。

1. `<progress>.tmp` を開く
2. `jsonencode(progData)` を書く
3. `movefile(...,'f')` で本体へ置換する

直接上書きではなくテンポラリ経由にしているため、中断時に空 JSON や途中書き込みが残る確率を下げています。  
長時間学習を前提とした checkpoint / progress 保護ロジックの一部です。

---

## `closeProgressFile(fid)`

`onCleanup` から呼ばれる単純な close helper です。  
`fid ~= -1` のときだけ `fclose` します。

小さい関数ですが、`writeProgressFile` の異常系でもファイルハンドルを確実に閉じるための役割があります。

---

## 読み解きポイント

- **実行本体**は `trainWithCurriculum`、**再開判定**は recovery 系 helper、**deployment 切替**は publish 系 helper、という責務分離になっています。
- フェーズごとの成果物は `phaseN_<track>` に閉じるので、途中停止後の検査がしやすい構成です。
- `progress.json` は run 全体の進捗、`AgentXX.mat` は phase 内の checkpoint、`final_agent.mat` は run 完了時点の代表成果物、という役割分担です。
