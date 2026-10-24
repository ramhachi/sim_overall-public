# `matlab/rl_driver/resumeAceDriverPhase4ExtensionAfterRestart.m`

## 役割

phase4 extension の再起動 bootstrap です。再開元 agent と deployment candidate の扱いを整理し、最後に `trainAceDriverPhase4Extension()` を起動します。

## 処理の流れ

1. `sourceAgentPath`（ace final）存在確認。
2. `deployment_agent.mat` があればそのまま維持。
3. 無ければ recovery final を fallback candidate として `publishRLDeploymentCandidate()` で publish。
4. `trainAceDriverPhase4Extension(...)` を既定 episode 設定で呼ぶ。

## システム上の位置づけ

再起動後の RL 継続学習を人手復旧なしで始めるための運用 helper です。
