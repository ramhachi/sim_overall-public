# `matlab/rl_driver/resolvePhase34RecoveryState.m`

## 役割

`resolvePhase34RecoveryState` は、phase 3/4 recovery を再開するときに、**どの checkpoint を再開元にし、どの phase から、何話残して、累積エピソード数をいくつとして続けるか**を決定する判定器です。

このファイルは recovery 再開整合性の司令塔であり、`trainPhase34Recovery` は自分で再開位置を判断せず、この返り値に従って `trainWithCurriculum` を呼び出します。

---

## 返り値 `state`

| フィールド | 意味 |
| --- | --- |
| `resume_agent_path` | 実際に再開へ使う Agent / checkpoint の path |
| `start_phase` | `trainWithCurriculum` に渡す開始 phase |
| `phase_episodes` | 再開後に各 phase で残っている episode 数 |
| `completed_episodes_offset` | run 全体の累積話数オフセット |
| `current_phase_completed_episodes` | 現在 phase 内で既に終わっていた話数 |
| `phase_resume_state_path` | `resume_state.json` の path |
| `mode` | 解決モード |
| `message` | 人間向け説明文 |

### `mode`

- `fresh_recovery`: 既存 run 状態を使わず新規 recovery として始める
- `resume_partial_phase`: 現在 phase の途中 checkpoint から残話数だけ再開する
- `resume_next_phase`: 直前 phase は完了済みなので、その最終 `agent.mat` から次 phase を始める

---

## 判定に使うファイル

### `progress.json`

run 全体の進捗です。  
少なくとも以下を参照します。

- `last_completed_phase`
- `completed_episodes`

### `phaseN_*/AgentXX.mat`

phase 内途中 checkpoint です。  
途中停止からの再開点探索に使います。

### `phaseN_*/training_stats.mat`

`EpisodeReward` の長さから、その phase で何話完了したかを推定する fallback 材料です。

### `phaseN_*/resume_state.json`

「途中 phase 再開を開始したセッション」の境界を記録する補助ファイルです。  
このファイルがあると、今回の再開セッションより前にあった古い checkpoint と、今回以降に増えた checkpoint を区別できます。

---

## 処理の大筋

1. まず既定状態を `fresh_recovery` として初期化する。
2. `SaveBaseDir/progress.json` が無ければ、そのまま新規 recovery 扱いにする。
3. `progress.json` があれば、最後に完了した phase と総話数を読む。
4. 既に全フェーズ完了済みならエラーにする。
5. 次に進行すべき active phase を決め、その phase directory を探す。
6. `resume_state.json` があれば、今回の再開セッション境界を使って新しい checkpoint を探す。
7. 適切な checkpoint が見つかり、かつその phase に残話数があるなら `resume_partial_phase` を返す。
8. そうでなく、直前 phase が完了済みで `agent.mat` があるなら `resume_next_phase` を返す。
9. どれにも当てはまらなければ `fresh_recovery` に戻る。
10. `CompletedEpisodesOffset` 未指定時は、再開元 checkpoint から自然な累積話数を推定する。

---

## 関数詳細

## `resolvePhase34RecoveryState(varargin)`

### 入力検証

- `ResumeAgentPath` 必須
- `SaveBaseDir` 必須
- `PhaseEpisodes` は 4 要素必須

### 初期値

開始時点では以下を仮定します。

- `resume_agent_path = ResumeAgentPath`
- `start_phase = 3`
- `phase_episodes = opts.PhaseEpisodes`
- `completed_episodes_offset = 0`
- `mode = 'fresh_recovery'`

つまり recovery 系のデフォルト出発点は「phase 3 から再開」です。

### `progress.json` がある場合

`last_completed_phase` と `completed_episodes` を取得し、次の active phase を `last_completed_phase + 1` から決めます。  
ただし 4 フェーズ完了済みなら「この run は終わっているので別保存先を使え」とエラーにします。

### active phase の途中再開判定

1. `findPhaseDir` で `phaseN_*` ディレクトリを探す
2. `getPhaseResumeState` で `resume_state.json` を読む
3. `resume_state.json` があれば `findLatestCheckpointAfter(activePhaseDir, session_started_datenum)` を使う
4. 見つからなければ `resume_source_checkpoint` を fallback とする
5. phase 内完了話数を `phase_episode_offset + latestCheckpointEpisode` で求める
6. 残話数 `remainingEpisodes` が正なら `resume_partial_phase` を返す

このとき、`completed_episodes_offset = completedEpisodes + totalPhaseEpisodesCompleted` になります。  
つまり run 全体の累積話数に「現在 phase で既に終わっていた分」を足した値を環境へ渡します。

### 次 phase への再開判定

途中再開で決まらなくても、`last_completed_phase` が 1 以上でまだ最終 phase 未満なら、直前 phase の `agent.mat` を探します。  
見つかればその Agent を再開元にし、`start_phase = last_completed_phase + 1` で `resume_next_phase` を返します。

### 新規 recovery fallback

`progress.json` が無い、phase directory が無い、checkpoint が無い、などで run-local 状態から安全な継続点が定まらない場合は、最初に渡された `ResumeAgentPath` を使った `fresh_recovery` に戻ります。

---

## `getNumericField(s, fieldName, defaultValue)`

struct から有限スカラー数値だけを取り出す helper です。  
欠落・型不一致・非有限値なら `defaultValue` を返します。

`progress.json` や `resume_state.json` の読み込み時に、壊れた値でロジック全体が崩れないようにする安全弁です。

---

## `findPhaseDir(baseDir, phaseIndex)`

`baseDir` 直下から `phase%d_*` に一致するディレクトリを探します。  
最初の一致を返すため、「1 phase につき 1 ディレクトリ」という運用前提があります。

返り値は以下です。

- `phaseDir`: 実 path
- `phaseName`: ディレクトリ名。メッセージ生成にも使う

---

## `findLatestCheckpoint(phaseDir)`

`phaseDir` 直下の `Agent*.mat` を走査し、ファイル名に埋め込まれた episode 番号が最大のものを返します。  
`Agent15.mat` のような名前から数字を抜き、最も大きい番号を phase 内最新 checkpoint とみなします。

返り値:

- `checkpointPath`
- `checkpointEpisode`

---

## `findLatestCheckpointAfter(phaseDir, minDatenum)`

`findLatestCheckpoint` と似ていますが、**ファイル更新時刻が `minDatenum` より新しい checkpoint だけ**を候補にします。

これが `resume_state.json` と組み合わさることで、途中再開前から残っていた古い `AgentXX.mat` を無視し、今回セッション以降に生成された checkpoint を優先できます。

---

## `inferCompletedEpisodesOffsetFromCheckpoint(agentPath)`

`CompletedEpisodesOffset` が明示されていない場合の推定器です。

### 流れ

1. `getPhaseContext` で checkpoint が属する phase directory と run directory を求める
2. `progress.json` があれば `completed_episodes` を読む
3. checkpoint より後段 phase の完了話数を差し引いて、「その checkpoint 時点までの総話数」を推定する
4. うまく推定できなければ `getCompletedEpisodesFromPhaseDir` に fallback する

この補助により、run-local の phase 構造が残っていれば、外部から総話数を手で渡さなくても自然なエピソード番号継続ができます。

---

## `getCompletedEpisodesFromPhaseDir(phaseDir)`

phase 内の完了話数を求める helper です。

優先順位は以下です。

1. `training_stats.mat` を開き、`EpisodeReward` を持つ struct の要素数を使う
2. 無ければ `findLatestCheckpoint` の episode 番号を使う

`training_stats.mat` があればより実完了話数に近く、無ければ `AgentXX.mat` 名から保守的に推定します。

---

## `getPhaseResumeState(phaseDir)`

`resume_state.json` を読み、以下を返します。

- `has_state`
- `phase_episode_offset`
- `session_started_datenum`
- `resume_source_checkpoint`

ファイルが無ければ `has_state=false` の既定 struct を返します。  
この関数により、途中再開境界を持つ phase と、そうでない phase を同じ上位ロジックで扱えます。

---

## `getPhaseContext(agentPath)`

checkpoint path から phase 文脈を逆算する helper です。

- `phaseDir = fileparts(agentPath)`
- `runDir = fileparts(phaseDir)`
- `phaseDir` 名が `phase(\d+)_` に一致すれば `phaseIndex` を返す

つまり `.../phase3_skidpad/Agent20.mat` のような path から、「phase 3 の checkpoint だ」と判定します。

---

## recovery state 解決の読み解きポイント

- `progress.json` は run 全体、`resume_state.json` は途中再開セッション境界、`AgentXX.mat` は phase 内 checkpoint、`training_stats.mat` は完了話数推定、という役割分担です。
- 部分再開では「phase の残話数」を計算し直して返すため、`trainWithCurriculum` 側は単に残りだけ学習すればよい形になります。
- phase 完了後の再始動と、phase 途中中断からの再始動を別モードに分けているのが安全性の核です。
