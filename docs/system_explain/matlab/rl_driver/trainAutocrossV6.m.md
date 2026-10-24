# `matlab/rl_driver/trainAutocrossV6.m`

## 役割

`trainMultiTrackV5` の Phase 2 までで得たエージェントを読み込み、autocross 中心に継続学習するスクリプトです。  
位置づけとしては、**多トラック事前学習済み agent を実戦コースへ寄せる専用ファインチューニング段階**です。

## 実行フロー

1. Phase 2 学習済み agent をロードする。
2. autocross を主軸にしつつ circle / circle_cw / skidpad を混ぜた環境を作る。
3. 学習オプションを設定する。
4. 継続学習する。
5. 学習結果を保存する。
6. actor を C++ 用にエクスポートする。
7. 環境を解放して終了する。

## section 詳細

### `%% Phase 2エージェントをロード`

```matlab
s = load(fullfile(pwd, '../../rl_results/multi_v5_phase2/trained_agent.mat'));
agent = s.agent;
```

- V5 の Phase 2 結果を直接読み込みます。
- ファイル形式は `trained_agent.mat` 前提で、内部変数名も `agent` を想定しています。
- ここでは新規 agent 作成を行わないため、V5 が事前条件です。

### `%% 環境作成（混合トラック: autocross中心 + circle復習）`

```matlab
env = FSAEEnvironment('autocross');
env.TrackPool = {'autocross', 'autocross', 'autocross', 'circle', 'circle_cw', 'skidpad'};
env.MaxEpisodeSteps = 8000;
```

- 基本環境名は `autocross` で、TrackPool でも autocross を 3 回入れて比重を高めています。
- 一方で `circle`、`circle_cw`、`skidpad` も混ぜ、忘却を抑えながら実戦コースへ寄せる構成です。
- `MaxEpisodeSteps = 8000` は autocross を最後まで走るための長め設定です。

```matlab
actInfo = getActionInfo(env);
```

- 行動次元を表示し、実行時の環境仕様を確認します。

### `%% 学習オプション`

保存先は `../../rl_results/autocross_v6` です。  
主な学習設定は以下です。

- `MaxEpisodes = 500`
- `MaxStepsPerEpisode = env.MaxEpisodeSteps`
- `ScoreAveragingWindowLength = 30`
- `StopTrainingCriteria = 'AverageReward'`
- `StopTrainingValue = 800`
- `SaveAgentCriteria = 'EpisodeReward'`
- `SaveAgentValue = 100`

autocross 完走と高報酬化を狙い、V5 より高めの停止基準を置いています。

### `%% 学習`

```matlab
trainingStats = train(agent, env, trainOpts);
```

- Phase 2 agent を初期値として継続学習します。
- 完全新規学習ではなく、基礎旋回能力を流用して autocross 適応を早める戦略です。

### `%% 保存`

- `trained_agent.mat`
- `training_stats.mat`

を `outDir` に保存します。  
学習済み agent の保管と、後段の解析用統計保存を分けています。

### `%% C++エクスポート`

- `rl_actor_export`
- `outDir`

へ actor を出力します。  
学習直後の推論反映まで一気通貫で行う構成です。

## このスクリプトの意味

- V5 で作った汎用性を残しつつ、autocross 成績を引き上げる専用段階です。
- curriculum 系の中では比較的分かりやすい「前段重みを読み込んで autocross に集中する」パターンの例になっています。
