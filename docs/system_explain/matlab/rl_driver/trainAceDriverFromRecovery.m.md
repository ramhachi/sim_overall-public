# `matlab/rl_driver/trainAceDriverFromRecovery.m`

## 役割

`trainAceDriverFromRecovery` は、`phase34_recovery_from_phase3_final/final_agent.mat` を起点に **より高速な autocross 指向へ継続学習する専用ラッパ**です。  
既存の `trainWithCurriculum` と `resolvePhase34RecoveryState` を再利用し、完走重視 recovery から「速いエースドライバー」寄りの phase 3/4 続行へ切り替えます。

## 処理の流れ

1. `ResumeAgentPath` と `SaveBaseDir` を project root 基準へ解決する。
2. 元 recovery run の `progress.json` から `completed_episodes` を読み、累計 episode オフセット候補を作る。
3. `resolvePhase34RecoveryState(...)` で ace run 側の `progress.json` / `Agent*.mat` / `resume_state.json` を見ながら安全な再開点を決める。
4. `loadRLAgentFromFile(...)` で再開元 agent を読む。
5. 速度寄りの `envOverrides`、review 付き `phaseTrackPools`、phase 3/4 用 reward override を構築する。
6. phase 途中再開なら `resume_state.json` を更新する。
7. `startRLServer(projectRoot)` のあと `trainWithCurriculum('autocross', ...)` を起動する。

## ace-driver 向けの固定化内容

### 保存先

- 再開元既定値: `rl_results/curriculum_torque_frame_fresh_v1/phase34_recovery_from_phase3_final/final_agent.mat`
- 出力先既定値: `rl_results/curriculum_torque_frame_fresh_v1/phase34_ace_driver_from_phase3_final`

recovery run と ace run を別ディレクトリに分けることで、完走重視の成果物を壊さずに高速化実験を進められます。

### エピソード設定

- `PhaseEpisodes = [1, 1, 360, 2160]`
- `PhaseSteps = [500, 1500, 4500, 6500]`

実質的には phase 3 / 4 続行で、phase 4 に長く学習リソースを割いています。

### 環境 override

主な上書きは次です。

- `CrawlWindowSteps = 80`
- `CrawlMinProgress = 5.0`
- `W_lowSpeedBrake = 16.0`
- `LowSpeedBrakeThreshold = 3.5`
- `W_previewSteer = 18.0`
- `PreviewSteerSpeedThreshold = 12.0`
- `PreviewCurvatureThreshold = 0.012`
- `RollingStartSpeed = 6.0`
- `RollingStartThrottle = 0.45`
- `RollingStartMaxSteps = 60`
- `RollingStartHoldEpisodes = 120`
- `RollingStartFadeEpisodes = 360`

低速で止まりがちな保守挙動を抑え、序盤だけ rolling start を使って周回中の速度形成へ学習を寄せる設計です。

### トラックプール

- phase 3: `skidpad`, `figure_eight`, `circle_R20`, `circle_R15`, `straight_corner`, `autocross`
- phase 4: `autocross` を厚くしつつ `figure_eight`, `skidpad`, `circle_R20`, `straight_corner` を review として残す

つまり「autocross だけ」ではなく、速く走るためのコーナリング要素を維持する mixed-track continuation です。

### reward override

- `W_speed` を上げる
- `W_boundary` / `W_centering` / `W_heading` は recovery より少し緩める
- `W_lap` を上げる
- phase 4 では `W_jerk` を少し下げる

完走最優先から、速さ寄りだが破綻しない方向へ重みを移しています。

## 補助関数

### `resolveProjectRelativePath(pathValue, projectRoot)`
相対 path を project root 基準へ揃える helper です。

### `inferAceSourceCompletedEpisodesOffset(agentPath)`
元 recovery run の `progress.json` から `completed_episodes` を拾い、ace run 開始時の episode index 連続性を保つ helper です。

### `writePhaseResumeState(statePath, phaseEpisodeOffset, resumeSourceCheckpoint)`
phase 途中再開の開始点を `resume_state.json` へ書き出します。途中停止後に古い checkpoint へ巻き戻らないための補助です。
