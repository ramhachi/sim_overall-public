# `matlab/rl_driver/trainPhase34Recovery.m`

## 役割

`trainPhase34Recovery` は、phase 4 学習で circle / skidpad 系スキルを忘れたあとに、**強い phase 3 系 Agent から mixed-track で回復学習を再開するための専用ラッパ**です。  
新しい学習器を実装しているのではなく、`trainWithCurriculum` へ recovery 用の再開点・環境オーバーライド・トラックプール・報酬重みを安全に注入する構成です。

---

## このファイルが解決する問題

phase 4 の autocross 学習は、軌道追従を広げる代わりに円旋回能力を忘却しやすいです。  
この関数は以下を固定化して、その忘却からの回復を狙います。

- 再開元は「安定した phase 3 系 checkpoint」
- preview steer shaping は常時有効寄り
- phase 3 / 4 に review 用トラックを混ぜる
- 長時間実行に備えて途中再開状態を別ファイルへ残す

---

## 処理の流れ

1. オプションを parse する。
2. `ResumeAgentPath` と `SaveBaseDir` を project root 基準へ正規化する。
3. `resolvePhase34RecoveryState` で最も安全な再開戦略を解決する。
4. `loadRLAgentFromFile` で実際の Agent をロードする。
5. recovery 専用の `envOverrides`、`phaseTrackPools`、`phaseRewardOverrides` を構築する。
6. 途中 phase 再開なら `resume_state.json` を更新する。
7. `startRLServer(projectRoot)` で RL server を起動・再接続可能状態へする。
8. `trainWithCurriculum('autocross', ...)` を recovery 条件付きで呼ぶ。

---

## recovery 専用のプリセット

### `envOverrides`

この関数は以下の環境プロパティを上書きします。

- `CrawlWindowSteps = 100`
- `CrawlMinProgress = 3.0`
- `W_lowSpeedBrake = 12.0`
- `LowSpeedBrakeThreshold = 2.0`
- `W_previewSteer = 25.0`
- `PreviewSteerSpeedThreshold = 0.0`
- `PreviewCurvatureThreshold = 0.01`

要点は、低速での詰まり対策と preview steer 系 shaping を強めに入れていることです。  
とくに `W_previewSteer` を高くし、速度閾値を 0 にしているため、曲率ベースの誘導が常時効きやすい設定になっています。

### `phaseTrackPools`

phase 1, 2 は空配列で既定値維持、phase 3, 4 だけ recovery 用に差し替えます。

- phase 3: `skidpad`, `figure_eight`, `circle_*`
- phase 4: `autocross` を主にしつつ `skidpad`, `figure_eight`, `circle_*`, `straight_corner`

この混合により、autocross へ伸ばしながら circle 系能力を再学習・維持しやすくします。

### `phaseRewardOverrides`

phase 3, 4 だけ境界逸脱とセンタリング系の重みを上げます。  
軌道維持を優先し、復旧初期の不安定挙動を抑える意図です。

---

## 関数詳細

## `trainPhase34Recovery(varargin)`

### 入力

主なオプションは以下です。

| 引数 | 役割 |
| --- | --- |
| `ResumeAgentPath` | recovery の基本再開元。既定は `phase3_phase4_from_phase2o/phase3_skidpad/agent.mat` |
| `SaveBaseDir` | recovery 実行先ディレクトリ |
| `CompletedEpisodesOffset` | 総エピソード数を外部から明示したい場合の override |
| `PhaseEpisodes` | 4 フェーズ分のエピソード数 |
| `PhaseSteps` | 4 フェーズ分の `MaxStepsPerEpisode` |
| `RandInterval`, `RandStrength` | DR 設定 |
| `SaveAgentValue` | `AgentXX.mat` 保存間隔 |
| `ScoreAveragingWindowLength` | 学習統計平均窓 |
| `Plots`, `Verbose` | RL Toolbox 学習表示 |

### 重要な挙動

- `PhaseEpisodes` と `PhaseSteps` は 4 要素必須です。
- `resolvePhase34RecoveryState` が返す `start_phase` / `phase_episodes` / `completed_episodes_offset` をそのまま `trainWithCurriculum` へ渡します。
- `loadRLAgentFromFile` で読んだ変数名自体は実行に使わず、Agent オブジェクトだけ再利用します。
- recovery 状態は標準出力へ詳しく表示されるため、運用者が「どこから再開したか」をログで確認しやすいです。

### `trainWithCurriculum` への橋渡し内容

この関数が下流へ渡す重要引数は以下です。

- `ResumeAgent`
- `SaveBaseDir`
- `PhaseEpisodes`（recovery state 反映済み）
- `PhaseSteps`
- `EnvOverrides`
- `PhaseTrackPools`
- `PhaseRewardOverrides`
- `StartPhase`
- `CompletedEpisodesOffset`
- `DomainRand`, `RandInterval`, `RandStrength`
- `SaveAgentCriteria='EpisodeCount'`
- `StopTrainingCriteria='AverageReward'`, `StopTrainingValue=1e9`

`StopTrainingValue=1e9` は、平均報酬で早期停止しない運用を意図した非常に高い値です。

---

## `resolveProjectRelativePath(pathValue, projectRoot)`

相対 path を project root 基準の絶対 path へ変換する helper です。

- Unix 絶対 path (`/`) はそのまま
- Windows 風 drive path (`C:\...`) もそのまま
- それ以外は `fullfile(projectRoot, pathStr)` で解決

recovery 実行時に MATLAB の current folder に依存しないよう、保存先と再開元を project 基準へ固定します。

---

## `writePhaseResumeState(statePath, phaseEpisodeOffset, resumeSourceCheckpoint)`

途中 phase 再開の開始時点を `resume_state.json` へ書く helper です。

### 保存内容

- `phase_episode_offset`: その phase で既に完了していた話数
- `resume_source_checkpoint`: 今回の再開元 checkpoint
- `session_started_datenum`: 今回の再開セッション開始時刻
- `updated_at`: 人間可読の更新時刻

### なぜ必要か

途中 phase を再開したあと、さらに途中で停止すると「この phase の最初からある古い `AgentXX.mat`」と「今回の再開後に増えた新しい `AgentXX.mat`」が混在します。  
`resolvePhase34RecoveryState` はこのファイルの `session_started_datenum` を使い、**今回の再開セッション以降に生成された checkpoint だけ**を優先的に拾います。

これにより、途中再開後に古い checkpoint へ巻き戻る事故を避けています。

---

## チェックポイント handling の要点

- 基本再開元は `ResumeAgentPath`
- ただし実際の再開点は `resolvePhase34RecoveryState` が `progress.json`、`resume_state.json`、`AgentXX.mat`、`training_stats.mat` を見て再解決する
- 部分再開時には `resume_state.json` を更新し、次回停止後の checkpoint 探索基準にする
- phase 完了済みなら直前 phase の `agent.mat` を使って次 phase へ進める

つまり `ResumeAgentPath` は「初期候補」であり、長時間 recovery の途中再開では run-local 実態が優先されます。
