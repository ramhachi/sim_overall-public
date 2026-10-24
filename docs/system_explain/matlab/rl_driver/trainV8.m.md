# `matlab/rl_driver/trainV8.m`

## 役割

長時間カリキュラム学習と報酬最適化を組み合わせた高性能化スクリプトです。  
V7 より長い訓練時間と厳しめの評価指標を使い、**円の基礎 → 複合旋回 → autocross 中心の実戦訓練**をさらに強化します。

## V7 から読み取れる発展点

- 各フェーズのエピソード数が増えている。
- Phase 3 の停止閾値が非常に高く、より高報酬な運転を狙う。
- 各フェーズ終了時に `last20avg` を表示し、学習状態を簡易分析している。

## section 詳細

### `%% Phase 1: 多様な円形コース`

```matlab
env.TrackPool = {'circle', 'circle_cw', ...
    'circle_R20', 'circle_cw_R20', ...
    'circle_R15', 'circle_cw_R15', ...
    'circle_R10', 'circle_cw_R10'};
agent = createSACAgent(env);
```

- 多様な半径の円を使い、基礎操舵能力を広い曲率範囲へ拡張します。
- ここで新規 agent を生成し、以後全フェーズで引き継ぎます。

学習設定:

- `MaxEpisodes = 400`
- `ScoreAveragingWindowLength = 30`
- `StopTrainingValue = 800`
- `SaveAgentValue = 200`

学習後は `stats1.EpisodeReward` を `R1` に取り出し、最大報酬と最後 20 エピソード平均を表示します。

### `%% Phase 2: スキッドパッド + 複合`

```matlab
env2.TrackPool = {'circle', 'circle_cw', ...
    'circle_R15', 'circle_cw_R15', ...
    'circle_R10', 'circle_cw_R10', ...
    'skidpad', 'skidpad', ...
    'straight_corner'};
```

- `skidpad` を 2 回入れて比重を高めています。
- `straight_corner` を混ぜ、単純な定常旋回だけでなく遷移動作も学ばせます。

学習設定:

- `MaxEpisodes = 300`
- `StopTrainingValue = 600`
- `SaveAgentValue = 100`

Phase 1 よりやや難しい内容ですが、停止閾値は少し低めにして、遷移期の学習を進めやすくしています。

### `%% Phase 3: オートクロス中心 + 復習`

```matlab
env3 = FSAEEnvironment('autocross');
env3.TrackPool = {'autocross', 'autocross', 'autocross', ...
    'circle', 'circle_cw', ...
    'circle_R10', 'circle_cw_R10', ...
    'skidpad'};
env3.MaxEpisodeSteps = 8000;
```

- 基本環境を `autocross` に変え、最終目標をはっきり示しています。
- TrackPool でも autocross を 3 回入れて比重を高めます。
- Phase 3 の設定は明確に実戦性能重視です。

学習設定:

- `MaxEpisodes = 600`
- `ScoreAveragingWindowLength = 50`
- `StopTrainingValue = 3000`
- `SaveAgentValue = 500`

特に `StopTrainingValue = 3000` はかなり高く、十分に高報酬へ到達するまで粘る構成です。

### `%% エクスポート`

```matlab
exportActorForCpp(agent, fullfile(pwd, 'rl_actor_export'));
exportActorForCpp(agent, fullfile(pwd, '../../cpp_export'));
```

- actor を RL 用共通出力先と `../../cpp_export` の 2 箇所へ出力します。
- V8 以降は C++ 側連携を強く意識した構成になっています。

## このスクリプトの読み解き

- V7 はトラック難易度の段階設計が中心でしたが、V8 は訓練時間と目標報酬も大きく引き上げています。
- ソースコメントにある通り「高性能学習」を掲げる版で、完走だけでなく高得点化も重視しています。
