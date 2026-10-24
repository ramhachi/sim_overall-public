# `matlab/rl_driver/trainMultiTrackV5.m`

## 役割

circle、circle_cw、skidpad、autocross を段階的に学習させる 3 フェーズ型スクリプトです。  
単一トラック学習から一歩進めて、**易しい円学習 → スキッドパッド追加 → オートクロス追加**の順で policy を育てます。

## 全体構成

- **Phase 1**: 双方向円での事前学習
- **Phase 2**: スキッドパッドを混ぜた継続学習
- **Phase 3**: オートクロスを含む混合トラック学習
- 最後に C++ へ actor をエクスポート

このファイルは curriculum の原型に近く、後続の V6〜V10 より単純です。

## 実行フロー

1. Phase 1 で円の基礎旋回を習得する。
2. Phase 2 で skidpad を追加し、左右切り返しや複合旋回へ拡張する。
3. Phase 3 で autocross を追加し、実戦的コースへ適応させる。
4. 最終エージェントを保存し、C++ 用 actor を出力する。

## section 詳細

### `%% Phase 1: 円形コース学習（両方向）`

```matlab
env = FSAEEnvironment('circle');
env.TrackPool = {'circle', 'circle_cw'};
agent = createSACAgent(env);
```

- 初期フェーズでは双方向円のみを学びます。
- ここで新規 agent を生成し、以後のフェーズへ引き継ぎます。

学習設定は次の通りです。

- `MaxEpisodes = 200`
- `ScoreAveragingWindowLength = 20`
- `StopTrainingValue = 600`
- `SaveAgentValue = 100`

保存先は `../../rl_results/multi_v5_phase1` です。  
学習後、`stats1.EpisodeReward` の長さと最大値を表示して、フェーズ単体の進み具合を把握できるようにしています。

### `%% Phase 2: スキッドパッド学習（既存の重みから継続）`

```matlab
env2 = FSAEEnvironment('circle');
env2.TrackPool = {'circle', 'circle_cw', 'skidpad'};
```

- Phase 1 の `agent` をそのまま使います。新規生成しない点が重要です。
- 学習対象に `skidpad` を加え、円学習で得た基礎操舵を複合旋回へ広げます。

学習設定:

- `MaxEpisodes = 200`
- `StopTrainingValue = 500`
- `SaveAgentValue = 50`

Phase 1 より保存閾値が低く、フェーズ遷移直後の改善を拾いやすくしています。  
保存先は `../../rl_results/multi_v5_phase2` です。

### `%% Phase 3: オートクロス学習`

```matlab
env3 = FSAEEnvironment('circle');
env3.TrackPool = {'circle', 'circle_cw', 'skidpad', 'autocross'};
env3.MaxEpisodeSteps = 10000;
```

- ここで `autocross` を追加し、もっと長いエピソード長を許可します。
- `MaxEpisodeSteps = 10000` はコメント通り autocross 用に延長した設定です。
- 基本環境名は `circle` ですが、実際は `TrackPool` の混合訓練です。

学習設定:

- `MaxEpisodes = 300`
- `ScoreAveragingWindowLength = 30`
- `StopTrainingValue = 400`
- `SaveAgentValue = 50`

保存先は `../../rl_results/multi_v5_final` です。

### `%% C++エクスポート`

```matlab
exportActorForCpp(agent, fullfile(pwd, 'rl_actor_export'));
exportActorForCpp(agent, outDir3);
```

- 最終フェーズの agent を C++ 推論用に変換します。
- 作業用共通ディレクトリと、Phase 3 結果ディレクトリの両方へ出力しています。

## フェーズ間の状態継承

このスクリプトの核心は、**同じ `agent` 変数を 3 フェーズで使い回す**ことです。  
したがって各フェーズは独立学習ではなく、前段の重みを引き継いだ fine-tuning になっています。

## 生成される成果物

- `rl_results/multi_v5_phase1/trained_agent.mat`
- `rl_results/multi_v5_phase1/training_stats.mat`
- `rl_results/multi_v5_phase2/trained_agent.mat`
- `rl_results/multi_v5_phase2/training_stats.mat`
- `rl_results/multi_v5_final/trained_agent.mat`
- `rl_results/multi_v5_final/training_stats.mat`
- actor エクスポート成果物
