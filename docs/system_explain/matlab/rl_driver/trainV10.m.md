# `matlab/rl_driver/trainV10.m`

## 役割

コース幅 3 m 前提へ学習条件を合わせ直した、精度重視の 3 フェーズ学習スクリプトです。  
冒頭コメントの通り、V9 で生じた **学習時コース幅と評価時コース幅のミスマッチ**を補正し、狭いコースで外さず走れる policy を目指します。

## V9 との差分として重要な点

- 境界ペナルティとセンタリング重みが全体的に強い。
- `W_slip`、`W_spin`、`W_jerk` も明示設定され、挙動の荒さを抑える方向が見える。
- Phase 1 をゼロから再学習している。

## 実行フロー

1. Phase 1 で基礎操作を 3 m 幅想定で学習する。
2. Phase 2 で autocross 中心へ広げる。
3. Phase 3 で精度を保ちながら速度向上を狙う。
4. 最終 actor をエクスポートする。

## section 詳細

### `%% Phase 1: 基礎操作（500ep, 3m幅）`

```matlab
env = FSAEEnvironment('circle');
env.TrackPool = {'circle', 'circle_cw', ...
    'circle_R20', 'circle_cw_R20', ...
    'circle_R15', 'circle_cw_R15', ...
    'circle_R10', 'circle_cw_R10'};
env.MaxEpisodeSteps = 3000;
agent = createSACAgent(env);
```

- まず多様な円で基礎操作を再構築します。
- コメント通り、3 m 幅に合わせて狭いコース許容度を前提にした訓練です。

報酬重み:

- `W_progress = 8.0`
- `W_speed = 0.5`
- `W_boundary = 50.0`
- `W_centering = 5.0`
- `W_lap = 200.0`
- `W_heading = 3.0`
- `W_alive = 0.2`
- `W_slip = 0.5`
- `W_spin = 15.0`
- `W_jerk = 0.05`

境界・センタリング・スピン抑制が強く、まずは正確な走行ラインを覚えさせる設計です。

学習設定:

- `MaxEpisodes = 500`
- `ScoreAveragingWindowLength = 30`
- `StopTrainingValue = 1e9`
- `SaveAgentValue = 100`

### `%% Phase 2: オートクロス集中（1000ep）`

```matlab
env2 = FSAEEnvironment('autocross');
env2.TrackPool = {'autocross', 'autocross', 'autocross', ...
    'circle', 'circle_cw', ...
    'skidpad', ...
    'circle_R15', 'circle_cw_R15'};
env2.MaxEpisodeSteps = 8000;
```

- autocross を主軸にしつつ、基礎トラックも残した構成です。
- Phase 1 の agent を継続利用します。

報酬重みは次のバランスです。

- `W_progress = 10.0`
- `W_speed = 1.0`
- `W_boundary = 50.0`
- `W_centering = 4.0`
- `W_lap = 300.0`
- `W_heading = 3.0`
- `W_alive = 0.1`
- `W_slip = 0.5`
- `W_spin = 15.0`
- `W_jerk = 0.05`

Phase 1 より少し速度を許容しつつ、狭いコース幅への精度要求は維持しています。

学習設定:

- `MaxEpisodes = 1000`
- `ScoreAveragingWindowLength = 50`
- `SaveAgentValue = 200`

### `%% Phase 3: スピード最適化（500ep）— 精度を維持しつつ速度向上`

```matlab
env3 = FSAEEnvironment('autocross');
env3.TrackPool = {'autocross', 'autocross', 'autocross', ...
    'circle', 'circle_cw', 'skidpad'};
env3.MaxEpisodeSteps = 8000;
```

- 最終段では autocross 比重を維持したまま速度改善へ進みます。

報酬重み:

- `W_progress = 12.0`
- `W_speed = 1.5`
- `W_boundary = 45.0`
- `W_centering = 3.0`
- `W_lap = 400.0`
- `W_heading = 2.5`
- `W_alive = 0.05`
- `W_slip = 0.5`
- `W_spin = 15.0`
- `W_jerk = 0.03`

Phase 2 より速度寄りですが、境界やスピン抑制は十分高く保たれています。  
「速くしたいが 3 m 幅を外さない」がこの section の主題です。

### `%% エクスポート`

- `rl_actor_export`
- `../../cpp_export`

へ最終 agent を出力します。  
最後に各フェーズのエピソード数を合計表示し、総訓練量を把握できるようにしています。

## このファイルの読みどころ

- V9 が「速さを強く追う版」なら、V10 は「狭い幅での実用性を再優先した版」です。
- 報酬重みの変更がこの意図を最もよく表しています。
