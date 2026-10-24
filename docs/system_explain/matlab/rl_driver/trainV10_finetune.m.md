# `matlab/rl_driver/trainV10_finetune.m`

## 役割

V10 Phase 2 のエージェントを基に、横方向誤差をさらに抑えるための精度ファインチューニングを行うスクリプトです。  
コメントにある通り、目標は **横方向誤差 1.5 m 以下を意識しつつ速度を維持する**ことです。

## このスクリプトの位置づけ

- V10 本体 3 フェーズの後段微調整
- 速度最適化よりも精度回復・精度強化が主目的
- ただし速度報酬をゼロにせず、極端な鈍化は避ける

## 実行フロー

1. V10 Phase 2 学習済み agent を読み込む。
2. circle / skidpad / autocross 混合環境を作る。
3. 精度重視の報酬重みへ設定する。
4. 400 エピソード学習する。
5. 学習済み agent と統計を保存する。
6. C++ 用 actor を出力する。
7. 環境を解放する。

## 先頭直列処理の詳細

このファイルは `%%` block が末尾のエクスポート部にしかなく、冒頭から学習本体が直列に書かれています。  
可読性上は section 化されていませんが、処理は次の塊に分けて読むと理解しやすいです。

### 1. 学習開始メッセージ

```matlab
fprintf('=== v10 Precision Fine-tuning (400 eps) ===\n');
```

- この run がファインチューニング専用であることを表示します。

### 2. V10 Phase 2 エージェントのロード

```matlab
loaded = load('../../rl_results/v10_phase2/trained_agent.mat');
if isfield(loaded, 'agent')
    agent = loaded.agent;
else
    fn = fieldnames(loaded);
    agent = loaded.(fn{1});
end
```

- `v10_phase2` をベースとし、Phase 3 ではなく Phase 2 を基準にしています。
- これは速度最適化済みの agent より、精度寄りの phase2 agent を起点にした方が狙いに合うためと読めます。
- `agent` 変数名が違う場合にも対応しています。

### 3. 環境セットアップ

```matlab
env = FSAEEnvironment('circle');
env.TrackPool = {'circle', 'circle_cw', ...
    'circle_R15', 'circle_cw_R15', ...
    'skidpad', 'skidpad', ...
    'autocross', 'autocross', 'autocross'};
env.MaxEpisodeSteps = 8000;
```

- 基本環境名は `circle` ですが、実際には混合トラック訓練です。
- `autocross` を 3 回、`skidpad` を 2 回入れ、精度が崩れやすいコースへ比重を置いています。
- `circle` 系も残し、基礎旋回精度の再学習を兼ねます。

### 4. 精度最大化の報酬重み

```matlab
env.W_progress = 10.0;
env.W_speed = 0.8;
env.W_boundary = 80.0;
env.W_centering = 8.0;
env.W_lap = 300.0;
env.W_heading = 4.0;
env.W_alive = 0.1;
env.W_slip = 0.5;
env.W_spin = 15.0;
env.W_jerk = 0.05;
```

- `W_boundary = 80.0` と `W_centering = 8.0` が特に強く、コース中心維持を極めて重視しています。
- `W_heading = 4.0` も高く、向きの乱れを強く嫌う設定です。
- `W_speed = 0.8` を残しているため、遅く走ればよいという学習にはしていません。

### 5. 学習オプションと実行

保存先は `../../rl_results/v10_finetune` です。  
主な設定:

- `MaxEpisodes = 400`
- `ScoreAveragingWindowLength = 30`
- `StopTrainingValue = 1e9`
- `SaveAgentValue = 200`

```matlab
stats = train(agent, env, trainOpts);
```

- ここでも早期停止は実質無効です。
- 学習後は `R = stats.EpisodeReward` として統計を取り出し、最大報酬と最後 30 エピソード平均を表示します。

## `%% エクスポート`

```matlab
exportActorForCpp(agent, fullfile(pwd, 'rl_actor_export'));
exportActorForCpp(agent, fullfile(pwd, '../../cpp_export'));
```

- 最終 agent を C++ 用 actor に変換します。
- その後 `delete(env)` で環境を解放し、`Fine-tuning done.` を表示して終了します。

## このファイルの意味

- V10 系列の中でも「仕上げの精度調整」専用です。
- 狭い幅・厳しいトラックでの逸脱抑制を最優先するため、報酬重みがシリーズ中でもかなり厳しい部類です。
