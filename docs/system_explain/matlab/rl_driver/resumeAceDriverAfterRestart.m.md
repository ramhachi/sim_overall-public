# `matlab/rl_driver/resumeAceDriverAfterRestart.m`

## 役割

`resumeAceDriverAfterRestart` は、**ace-driver 継続学習の運用再開ショートカット**です。  
端末停止後も deployment を不用意に崩さず、ace run の `progress.json` / checkpoint を優先して再開する入口として使います。

## 処理内容

1. `rl_results/deployment/deployment_agent.mat` があれば保持する。
2. 無ければ安全側 fallback として `phase2o_circle_preview_always_on/Agent19.mat` を deployment へ公開する。
3. `trainAceDriverFromRecovery(...)` を既定プリセットで呼ぶ。

## `resumeAceDriverAfterRestart()`

### deployment 保護

ace continuation は未検証の高速化 run なので、既存 deployment を即座に上書きしません。  
deployment が空のときだけ、既知の fallback を入れてから継続学習へ進みます。

### 既定で渡す条件

- `PhaseEpisodes = [1, 1, 360, 2160]`
- `ScoreAveragingWindowLength = 40`
- `SaveAgentValue = 20`

実際の partial-phase resume 判定や `AgentXX.mat` 探索は下流の `trainAceDriverFromRecovery` が行います。
