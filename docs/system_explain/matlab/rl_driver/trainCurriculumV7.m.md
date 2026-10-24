# `matlab/rl_driver/trainCurriculumV7.m`

## 役割

難易度を段階的に上げる 3 フェーズのカリキュラム学習スクリプトです。  
大半径円から始め、よりタイトな旋回や skidpad、最後に autocross へ進むことで、**基礎操舵 → 複合旋回 → 実戦コース**という順に能力を積み上げます。

## フェーズ設計

- **Phase 1**: 大半径円のみで基本操作を習得
- **Phase 2**: 小半径円・S 字系・skidpad で旋回難度を上げる
- **Phase 3**: autocross を主題にしつつ過去トラックを混ぜて復習する

## 実行フロー

1. Phase 1 で新規 agent を作成し、大半径円で学習する。
2. 同じ agent を Phase 2 へ持ち越し、タイトコーナや skidpad を混ぜて継続学習する。
3. 同じ agent を Phase 3 へ持ち越し、autocross 比重を高めて学習する。
4. 最終 agent を保存し、C++ 用へエクスポートする。

## section 詳細

### `%% Phase 1: 基本操作（大半径円、CW/CCW）`

```matlab
env = FSAEEnvironment('circle');
env.TrackPool = {'circle', 'circle_cw', 'circle_R20', 'circle_cw_R20'};
agent = createSACAgent(env);
```

- 大半径の円だけを使い、まず車を落ち着いて曲げる能力を学ばせます。
- `circle` と `circle_R20` の両方向版を含めており、向き依存も減らします。

学習設定:

- `MaxEpisodes = 200`
- `ScoreAveragingWindowLength = 20`
- `StopTrainingValue = 600`
- `SaveAgentValue = 100`

保存先は `../../rl_results/curriculum_v7_p1` です。

### `%% Phase 2: 多様なコーナリング（タイト＋S字＋スキッドパッド）`

```matlab
env2.TrackPool = {
    'circle', 'circle_cw',
    'circle_R20', 'circle_cw_R20',
    'circle_R15', 'circle_cw_R15',
    'circle_R10', 'circle_cw_R10',
    'circle_R7', 'circle_cw_R7',
    'skidpad', 'straight_corner'
};
```

- Phase 1 の agent をそのまま継続利用します。
- 小半径円を追加し、よりきつい旋回を学ばせます。
- `skidpad` と `straight_corner` を入れることで、単一曲率の円から一歩進んだ複合操作へ移ります。

学習設定:

- `MaxEpisodes = 300`
- `ScoreAveragingWindowLength = 30`
- `StopTrainingValue = 400`
- `SaveAgentValue = 50`

保存先は `../../rl_results/curriculum_v7_p2` です。

### `%% Phase 3: オートクロス中心 + 復習`

```matlab
env3 = FSAEEnvironment('circle');
env3.TrackPool = {'autocross', 'autocross', 'autocross', 'autocross', ...
    'circle', 'circle_cw', 'skidpad', ...
    'circle_R10', 'circle_cw_R10', 'straight_corner'};
env3.MaxEpisodeSteps = 8000;
```

- TrackPool の先頭に `autocross` を 4 回並べ、フェーズの主目的を autocross 適応に置いています。
- ただし旧課題を少量残し、学習済み技能の忘却を抑えます。
- `MaxEpisodeSteps = 8000` に増やし、長いコースへ対応させます。

学習設定:

- `MaxEpisodes = 500`
- `ScoreAveragingWindowLength = 30`
- `StopTrainingValue = 600`
- `SaveAgentValue = 50`

保存先は `../../rl_results/curriculum_v7_final` です。  
ここでは `trained_agent.mat` に加えて `training_stats.mat` として `stats3` も保存します。

### `%% エクスポート`

```matlab
exportActorForCpp(agent, fullfile(pwd, 'rl_actor_export'));
exportActorForCpp(agent, outDir3);
```

- 最終フェーズの agent を actor 形式に変換します。
- 共通エクスポート先と、最終学習ディレクトリの両方へ出力しています。

## このスクリプトの特徴

- curriculum の意図がソース上で最も明快な版の一つです。
- フェーズごとにコースの難易度と多様性を明示的に増やしています。
- 報酬重みそのものは変えず、主にトラック構成と学習長で段階制御している点が V8 以降との違いです。
