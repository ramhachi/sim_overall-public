# `matlab/rl_driver/trainCircleV4.m`

## 役割

時計回り・反時計回りの両方向円形コースを使って学習するスクリプトです。  
`trainCircleV3` の単方向学習を拡張し、**同じ円でも旋回方向が反転したケースへ一般化**させることを狙っています。

## `trainCircleV3` との主な違い

- `TrackPool = {'circle', 'circle_cw'}` を使う。
- エピソード数が `300` から `500` に増えている。
- 停止閾値が `AverageReward=600` に引き上げられている。

## 実行フロー

1. circle 環境を作る。
2. `TrackPool` に CW / CCW の 2 コースを設定する。
3. 行動次元 2 を検証する。
4. SAC エージェントを新規作成する。
5. 学習オプションを設定する。
6. 学習する。
7. 学習済み agent と統計を保存する。
8. C++ 用 actor を出力する。
9. 環境を解放する。

## section 詳細

### `%% 環境作成（2D行動空間、CW/CCWランダム選択）`

```matlab
env = FSAEEnvironment('circle');
env.TrackPool = {'circle', 'circle_cw'};
```

- 基本環境は `circle` ですが、実際のエピソード選択は `TrackPool` に委ねます。
- コメントから、各エピソードで `circle` と `circle_cw` がランダムに選ばれる想定です。
- 片方向専用学習で生じやすい「右回りだけ得意」「左回りだけ苦手」を減らす設計です。

```matlab
actInfo = getActionInfo(env);
obsInfo = getObservationInfo(env);
assert(actInfo.Dimension(1) == 2, 'Expected 2D action space');
```

- action 仕様の安全確認は V3 と同じです。
- `obsInfo` は取得のみで未使用です。

### `%% SAC エージェント作成`

```matlab
agent = createSACAgent(env);
```

- 双方向円学習に対応した SAC agent を新規生成します。
- 事前学習済み重みの継承はありません。

### `%% 学習オプション`

保存先は `../../rl_results/circle_v4` です。  
主な設定値は次の通りです。

- `MaxEpisodes = 500`
- `ScoreAveragingWindowLength = 30`
- `StopTrainingValue = 600`
- `SaveAgentValue = 100`

V3 よりエピソード数と窓長が大きく、双方向化で難しくなった分だけ安定した平均評価を重視しています。

### `%% 学習`

```matlab
trainingStats = train(agent, env, trainOpts);
```

- `CW+CCW` を混ぜた環境で学習します。
- 片側旋回で学んだ policy をそのまま使うのではなく、最初から両方向を経験させる構成です。

### `%% 保存`

- `trained_agent.mat`
- `training_stats.mat`

を `outDir` に保存します。  
V3 と同じ命名規則で、比較しやすい成果物構成です。

### `%% C++エクスポート`

- `rl_actor_export`
- `outDir`

の 2 箇所へ actor を出力します。  
学習直後に C++ 側推論へ繋げるための section です。

### `%% クリーンアップ`

```matlab
delete(env);
```

- 環境オブジェクトを解放して終了します。

## このスクリプトの読みどころ

- 最小構成のまま、トラック多様性だけを増やした版です。
- 後続の multi-track / curriculum 系より軽量で、双方向円だけの効果検証に向いています。
