# `matlab/rl_driver/trainCircleV3.m`

## 役割

2D 行動空間を明示的に確認しながら、円形コースだけで SAC 学習を行うスクリプトです。  
`trainCircleSimple` よりも短く、**新規 agent 作成 → 学習 → 保存 → C++ エクスポート**の最小実行例として使えます。

## スクリプトの目的

- `FSAEEnvironment('circle')` の行動次元が 2 であることを明示確認する。
- 300 エピソードの比較的短い学習を行う。
- 学習後すぐに C++ 用 actor を出力する。

## 実行フロー

1. circle 環境を作る。
2. 行動空間が 2 次元であることを `assert` で保証する。
3. SAC エージェントを新規作成する。
4. 学習オプションを設定する。
5. 学習統計を取得しながら学習する。
6. 学習済み agent と統計を保存する。
7. actor を 2 か所へエクスポートする。
8. 環境オブジェクトを `delete` して終了する。

## section 詳細

### `%% 環境作成（2D行動空間を強制確認）`

```matlab
env = FSAEEnvironment('circle');
actInfo = getActionInfo(env);
obsInfo = getObservationInfo(env);
assert(actInfo.Dimension(1) == 2, 'Expected 2D action space');
```

- 学習対象の環境を `circle` 固定で構築します。
- `getActionInfo(env)` で action spec を取得し、一次元目の長さが 2 であることを確認します。
- ここでの 2D 行動空間はコメントから `[steer, accel]` 系の運用を想定していると読めます。
- `obsInfo` も取得していますが、このファイル内では後続で使われません。観測仕様確認の名残またはデバッグ用途です。

この section の意図は、環境設定ミスで 3 入力化したり action 定義が崩れた状態で学習を始めないことです。

### `%% SAC エージェント作成`

```matlab
agent = createSACAgent(env);
```

- 環境仕様に基づいて SAC エージェントを新規作成します。
- 再開学習や checkpoint 読み込みは行わず、常にゼロから開始します。

### `%% 学習オプション`

```matlab
outDir = fullfile(pwd, '../../rl_results/circle_v3');
trainOpts = rlTrainingOptions(...);
```

- 保存先は `../../rl_results/circle_v3` です。
- 存在しなければ `mkdir` します。

設定値は以下です。

- `MaxEpisodes = 300`
- `MaxStepsPerEpisode = env.MaxEpisodeSteps`
- `ScoreAveragingWindowLength = 20`
- `StopTrainingCriteria = 'AverageReward'`
- `StopTrainingValue = 500`
- `SaveAgentCriteria = 'EpisodeReward'`
- `SaveAgentValue = 100`
- `SaveAgentDirectory = outDir`

つまり、短い窓長で平均報酬の立ち上がりを見ながら、一定以上のエピソード報酬を得た時点で中間保存する設定です。

### `%% 学習`

```matlab
trainingStats = train(agent, env, trainOpts);
```

- 300 エピソード上限で学習します。
- `trainingStats` に報酬推移などが格納されます。
- ログ文から、この section は「2D action の circle 学習」が正しく動くことを確認するための短期実行として書かれています。

### `%% 保存`

```matlab
save(fullfile(outDir, 'trained_agent.mat'), 'agent');
save(fullfile(outDir, 'training_stats.mat'), 'trainingStats');
```

- 学習済みエージェントと統計を `.mat` で保存します。
- 出力先は学習オプションで使った `outDir` と一致します。

### `%% C++エクスポート`

```matlab
exportActorForCpp(agent, fullfile(pwd, 'rl_actor_export'));
exportActorForCpp(agent, outDir);
```

- 同じ actor を 2 箇所へ出力しています。
- `rl_actor_export` は作業ディレクトリ直下の標準的なエクスポート先です。
- `outDir` にも出すことで、学習成果物とエクスポート成果物を同じバージョン単位で保管できます。

### `%% クリーンアップ`

```matlab
delete(env);
```

- 環境オブジェクトを明示解放します。
- サーバー接続や内部リソースを持つ環境クラスである可能性を考慮した後始末です。

## このスクリプトが生成する成果物

- `rl_results/circle_v3/trained_agent.mat`
- `rl_results/circle_v3/training_stats.mat`
- `rl_actor_export` 配下の actor 出力
- `rl_results/circle_v3` 配下の actor 出力

## 位置づけ

- 円形コース専用の簡易バージョン
- 2D action の確認を強調した最小サンプル
- curriculum 系の複雑な学習より前に、単純な訓練経路を試したいときの比較基準
