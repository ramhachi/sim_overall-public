# `matlab/rl_driver/trainV9.m`

## 役割

V8 Phase 3 のエージェントを初期値に使い、autocross の走行品質をさらに高める高性能最適化スクリプトです。  
特徴は、**早期停止を実質無効化した長時間継続学習**と、フェーズごとに異なる報酬重みで「速度重視 → 精度重視 → 速度最適化」を順に行う点です。

## 冒頭コメントが示す改善点

1. 早期停止なし
2. V8 Phase 3 agent から再開
3. 速度ボーナス強化
4. 境界ペナルティの微調整
5. 2000 エピソード超の徹底学習

## 実行フロー

1. V8 Phase 3 の agent をロードする。
2. Phase 1 で autocross 集中訓練を行う。
3. Phase 2 で精度重視の報酬重みに切り替える。
4. Phase 3 で再び速度最適化へ寄せる。
5. 最終 agent を保存し、C++ へエクスポートする。

## section 詳細

### `%% Phase 1: オートクロス集中訓練（800ep）— v8 Phase3から継続`

```matlab
env = FSAEEnvironment('autocross');
env.TrackPool = {'autocross', 'autocross', 'autocross', 'autocross', ...
    'circle', 'circle_cw', 'skidpad'};
env.MaxEpisodeSteps = 8000;
```

- autocross を 4 回入れ、最初から実戦コース偏重です。
- 基礎トラックも少量残して catastrophic forgetting を防ぎます。

報酬重みは速度重視です。

- `W_progress = 10.0`
- `W_speed = 1.5`
- `W_boundary = 35.0`
- `W_centering = 1.5`
- `W_lap = 300.0`
- `W_heading = 2.5`
- `W_alive = 0.1`

ここでは「多少攻めてもよいので前へ進み、速く走る」傾向を強めています。

```matlab
loaded = load('../../rl_results/v8_phase3/trained_agent.mat');
```

- 変数名が `agent` とは限らないケースに備え、
  - `agent`
  - `saved_agent`
  - その他の先頭フィールド
 という順で取り出しています。
- 過去成果物の保存形式ゆれに耐える読み込み処理です。

学習設定:

- `MaxEpisodes = 800`
- `ScoreAveragingWindowLength = 50`
- `StopTrainingValue = 1e9`
- `SaveAgentValue = 500`

`1e9` は実質「早期停止しない」ための設定です。

### `%% Phase 2: 精度強化（600ep）— 横方向誤差を減らす`

```matlab
env2.TrackPool = {'autocross', 'autocross', ...
    'circle', 'circle_cw', ...
    'circle_R15', 'circle_cw_R15', ...
    'skidpad'};
```

- autocross を主軸にしつつ、小半径円や skidpad も混ぜています。
- ここでは速度一辺倒ではなく、横方向誤差抑制が目的です。

報酬重みは精度寄りに再調整されます。

- `W_speed = 1.0` に低下
- `W_boundary = 50.0` に強化
- `W_centering = 4.0` に強化
- `W_heading = 3.0` に強化

すなわち、Phase 1 で得た攻めの運転をいったん引き締める段階です。

学習設定:

- `MaxEpisodes = 600`
- `StopTrainingValue = 1e9`
- `SaveAgentValue = 500`

### `%% Phase 3: 速度最適化（800ep）— ラップタイム短縮`

```matlab
env3.TrackPool = {'autocross', 'autocross', 'autocross', ...
    'circle', 'circle_cw', 'skidpad'};
```

- 再度 autocross 中心に戻し、ラップ短縮を狙います。
- Phase 2 より復習トラック構成を少し絞っています。

報酬重み:

- `W_progress = 12.0`
- `W_speed = 2.0`
- `W_boundary = 40.0`
- `W_centering = 2.0`
- `W_lap = 500.0`
- `W_heading = 2.0`
- `W_alive = 0.05`

Phase 2 より速度寄りですが、境界ペナルティは完全には緩めず、攻め過ぎで崩れないよう折衷しています。

### `%% エクスポート`

- `rl_actor_export`
- `../../cpp_export`

へ最終 agent を出力します。  
最後に総エピソード数 `800 + 600 + 800` を表示し、長時間学習の規模をまとめています。

## このスクリプトの本質

- 1 本の学習ではなく、**報酬設計を変えながら同一 agent を再調整する最適化パイプライン**です。
- V8 で完成した基礎性能を、速度と精度のトレードオフ調整で仕上げる版と読めます。
