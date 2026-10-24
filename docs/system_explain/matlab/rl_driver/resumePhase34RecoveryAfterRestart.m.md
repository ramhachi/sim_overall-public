# `matlab/rl_driver/resumePhase34RecoveryAfterRestart.m`

## 役割

`resumePhase34RecoveryAfterRestart` は、端末再起動・ MATLAB 再起動・長時間学習中断のあとに、**安全側 deployment を守りつつ phase 3/4 recovery 学習を再始動する運用用ブートストラップ**です。

このファイルは recovery ロジック本体ではなく、

1. deployment 候補の保護
2. fallback 候補の公開
3. recovery 実行開始

を 1 コマンドにまとめた入口です。

---

## 処理の流れ

1. `projectRoot` を `mfilename('fullpath')` から 3 階層上へたどって求める。
2. 既定 deployment path `rl_results/deployment/deployment_agent.mat` を決める。
3. fallback 候補 `rl_results/curriculum_torque_frame_fresh_v1/phase2o_circle_preview_always_on/Agent19.mat` を決める。
4. deployment 候補が既に存在すればそれを保持する。
5. 存在しなければ fallback Agent を `publishRLDeploymentCandidate` で deployment に公開する。
6. その後 `trainPhase34Recovery(...)` を長時間設定で起動する。

---

## なぜ deployment を先に保護するのか

recovery 学習は本質的に試行錯誤であり、再開直後の Agent をそのまま運用系へ出すべきではありません。  
この関数はまず `deployment_agent.mat` の有無を見て、既に安定候補があるなら**一切触らず保持**します。

つまり recovery run の再開と、C++ 実行系が参照する deployment 候補の切替を分離しています。

---

## fallback 候補の意味

既定 fallback は `phase2o_circle_preview_always_on/Agent19.mat` です。  
コードコメント上は「current safest fallback deployment candidate」とされており、少なくとも recovery 開始前に deployment が空になる事態を避ける安全弁として使われます。

公開時には `Metadata` として次を付与します。

- `published_reason = 'safe_fallback_before_phase34_recovery'`
- `published_by = mfilename`

そのため、`deployment_metadata.json` を見れば「recovery 前の safety publish だった」ことを後から追跡できます。

---

## 関数詳細

## `resumePhase34RecoveryAfterRestart()`

### 入力

引数はありません。  
運用者が迷わず再開できるよう、パス・fallback・学習長をすべて固定しています。

### `trainPhase34Recovery` へ渡す設定

- `PhaseEpisodes = [1, 1, 480, 1440]`
- `ScoreAveragingWindowLength = 30`
- `SaveAgentValue = 20`

phase 1, 2 を 1 episode にしているのは、実質的に後半 recovery を伸ばす意図です。  
主戦場は phase 3, 4 であり、長時間 mixed-track recovery を行うための設定になっています。

### 標準出力

関数は以下を必ず表示します。

- recovery bootstrap 開始メッセージ
- deployment を保持したか、fallback を publish したか
- 長時間 phase 3/4 recovery 開始メッセージ

このため、運用ログだけでも復旧時の判断履歴を追いやすいです。

---

## このファイルのシステム上の位置付け

- **本体学習**: `trainWithCurriculum`
- **recovery 条件の注入**: `trainPhase34Recovery`
- **recovery 再開点の解決**: `resolvePhase34RecoveryState`
- **運用上の再始動入口**: `resumePhase34RecoveryAfterRestart`

つまり本ファイルは、技術的には薄いラッパですが、実運用では「どの順番で安全に復旧するか」を固定する重要な入口です。
