# `matlab/rl_driver/trainAceDriverPhase4Extension.m`

## 役割

完走済み ace final agent を起点に、phase4 だけを追加学習する continuation entrypoint です。新しい save directory へ書き、進捗が残っていれば途中 checkpoint から再開します。

## 入力オプション

- `ResumeAgentPath`
- `SaveBaseDir`
- `CompletedEpisodesOffset`
- `Phase4Episodes`
- `Phase4Steps`
- `RandInterval`
- `RandStrength`
- `SaveAgentValue`
- `ScoreAveragingWindowLength`

## 全体フロー

1. `ensure_rl_driver_paths()`。
2. project root と path を解決。
3. `inferSourceCompletedEpisodesOffset()` で progress.json から offset を推定。
4. `resolvePhase34RecoveryState()` で fresh / partial resume を吸収。
5. `loadRLAgentFromFile()` で再開 agent を読む。
6. phase4 専用の `envOverrides`、`phaseTrackPools`、`phaseRewardOverrides` を組む。
7. partial resume なら `writePhaseResumeState()` を書く。
8. `startRLServer()` 後に `trainWithCurriculum()` を phase4-only 相当で実行。

## 設計意図

「phase34 ace 完走後に、mixed-track phase4 を追加で磨く」ことだけに責務を絞っています。既存の full curriculum を複製せず、state resolver を再利用して運用再開性を確保しています。
