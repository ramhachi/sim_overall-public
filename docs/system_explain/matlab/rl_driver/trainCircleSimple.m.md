# `matlab/rl_driver/trainCircleSimple.m`

## 役割

円形コース完走を最優先にした最小構成の学習関数です。  
多段カリキュラムや複数トラック混合を使わず、**circle 専用・2D 行動空間・保守的な報酬設定**で、まず RL ドライバーの成立を確認するためのベースラインを提供します。

## 関数シグネチャ

```matlab
trainCircleSimple(varargin)
```

### 主な名前付き引数

- `MaxEpisodes`  
  最大学習エピソード数。既定値は `100000`。
- `SaveDir`  
  結果保存先。既定値は `rl_results/circle_v2`。
- `ResumeAgent`  
  既存エージェントを渡して再開学習するための引数。
- `StopReward`  
  `AverageReward` による停止閾値。既定値は `500`。

### 戻り値

戻り値はありません。  
学習済みエージェントと統計を `.mat` として保存します。

## 実行フロー

1. `inputParser` でオプションを解釈する。
2. 保存先ディレクトリを作成する。
3. `FSAEEnvironment('circle')` を構築し、報酬重みと最大ステップ数を円形コース向けに調整する。
4. 新規 SAC エージェントを作るか、既存エージェントから再開する。
5. `rlTrainingOptions` を設定する。
6. `train` を呼んで学習する。
7. エージェントと学習統計を保存する。
8. 学習後に `evaluateRLDriver` で簡易評価を試みる。

## セクション詳細

### 関数ヘッダと先頭コメント

コメントにはこの関数の設計意図がまとまっています。

- カリキュラムなし
- 円形コースに特化
- 2D 行動空間 `[steer, accel]`
- 進行報酬を主とする報酬設計
- 探索を阻害しない緩めの終了条件

つまり、「まず走らせる」ことを最優先にした学習テンプレートです。

### `%% オプション解析`

```matlab
p = inputParser;
addParameter(...);
parse(p, varargin{:});
opts = p.Results;
```

- すべての入力は名前付き引数として受け取ります。
- `SaveDir` が存在しない場合は `mkdir` で作成します。
- この section で関数全体の動作モードが決まります。

特に重要なのは次の 2 点です。

1. **`ResumeAgent`**
   - 継続学習か新規学習かを分岐します。
2. **`StopReward`**
   - `AverageReward` の収束基準としてそのまま使われます。

### `%% 環境構築`

```matlab
env = FSAEEnvironment('circle');
env.MaxEpisodeSteps = 3000;
```

- コースは固定で `circle` です。
- 1 エピソードの最大ステップ数を `3000` にし、コメント上では約 60 秒相当を想定しています。

続いて報酬重みを個別に調整しています。

- `W_progress = 5.0`  
  進捗を主報酬として強く評価します。
- `W_speed = 0.3`  
  速度評価は弱めにし、まずは逸脱せず進むことを優先します。
- `W_boundary = 8.0`  
  コース外れの抑制です。
- `W_centering = 1.0`
- `W_heading = 0.5`
- `W_alive = 0.15`
- `W_lap = 200.0`  
  完走ボーナスをかなり大きくし、1 周完了を強く学習させます。
- `W_spin = 5.0`  
  スピン抑制です。

この section は「円で確実に完走できるように環境を簡略化する」部分です。

### `%% エージェント構築`

```matlab
if ~isempty(opts.ResumeAgent)
    agent = opts.ResumeAgent;
    assertAgentObservationDimension(...);
else
    agent = createSACAgent(env);
end
```

- `ResumeAgent` が与えられた場合はそのまま再利用します。
- ただし観測次元不一致で壊れないよう、`assertAgentObservationDimension` で検証します。
- 新規学習時は `createSACAgent(env)` により環境に対応した SAC エージェントを生成します。

再開時に観測設計が変わっていると学習不能になるため、ここは安全装置の役割もあります。

### `%% 学習オプション`

```matlab
trainOpts = rlTrainingOptions;
```

設定内容は次の通りです。

- `MaxEpisodes = opts.MaxEpisodes`
- `MaxStepsPerEpisode = env.MaxEpisodeSteps`
- `ScoreAveragingWindowLength = 50`
- `StopTrainingCriteria = 'AverageReward'`
- `StopTrainingValue = opts.StopReward`
- `SaveAgentCriteria = 'EpisodeReward'`
- `SaveAgentValue = opts.StopReward * 0.5`
- `SaveAgentDirectory = opts.SaveDir`
- `Plots = 'training-progress'`
- `Verbose = true`

特徴は、停止条件と保存条件が単純で分かりやすいことです。  
平均報酬で終了しつつ、その半分の水準を超えたエピソード報酬で中間エージェントを保存します。

### `%% 学習実行`

```matlab
trainingStats = train(agent, env, trainOpts);
```

- 学習開始前に停止条件や想定収束レンジを標準出力へ表示します。
- 実際の学習は MATLAB Reinforcement Learning Toolbox の `train` に委ねています。
- 返り値 `trainingStats` にはエピソード報酬などの履歴が入ります。

### `%% 結果保存`

```matlab
save(fullfile(opts.SaveDir, 'trained_agent.mat'), 'agent');
save(fullfile(opts.SaveDir, 'training_stats.mat'), 'trainingStats');
```

- 学習済み agent を `trained_agent.mat` に保存します。
- 学習履歴を `training_stats.mat` に保存します。
- 保存先はすべて `opts.SaveDir` 配下です。

保存後、簡易評価を実施します。

```matlab
try
    evaluateRLDriver(agent, 'circle');
catch e
    fprintf('Evaluation failed: %s\n', e.message);
end
```

- 評価失敗で本体を落とさないよう `try/catch` で包んでいます。
- 学習自体は成功していても評価系が失敗する可能性を切り分けるための構成です。

## この関数が作る成果物

- `SaveDir/trained_agent.mat`
- `SaveDir/training_stats.mat`

## 読み解きのポイント

- ベースライン確認用なので、汎用性より再現しやすさを優先しています。
- 複雑な curriculum 系の学習で問題が出たとき、まずこの関数に戻って基礎挙動を確認する使い方が自然です。
